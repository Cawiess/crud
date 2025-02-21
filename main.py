import sys
import os
import json
import uuid
import math
import copy
import re
import PyPDF2

from dataclasses import dataclass, asdict, field, fields
from typing import List, Dict, Any, Optional

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QGraphicsView, QGraphicsScene,
    QGraphicsItem, QGraphicsEllipseItem, QGraphicsPathItem, QGraphicsTextItem,
    QVBoxLayout, QHBoxLayout, QWidget, QFormLayout, QPushButton, QLineEdit,
    QLabel, QComboBox, QFileDialog, QToolBar, QAction, QTabWidget, QTextEdit,
    QInputDialog, QDialog, QGroupBox, QShortcut, QMessageBox, QCheckBox, QDockWidget, QPlainTextEdit,
    QSpinBox, QDoubleSpinBox
)
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QPainterPath, QPalette, QFont,
    QKeySequence, QTextCharFormat, QSyntaxHighlighter
)
from PyQt5.QtCore import Qt, QPointF, QTimer, QVariantAnimation

# =============================================================================
# DocumentHandler: Extracts persons and metadata from a PDF document.
# =============================================================================

class DocumentHandler:
    def __init__(self, pdf_path):
        self.document_id = os.path.basename(pdf_path)
        self.document_text = self.extract_text(pdf_path)

    def extract_text(self, pdf_path):
        with open(pdf_path, "rb") as pdf_file:
            reader = PyPDF2.PdfReader(pdf_file)
            full_text = []
            for page in reader.pages:
                page_text = page.extract_text()
                if page_text:
                    full_text.append(page_text)
        full_text = " ".join(full_text).replace("\n", " ")
        self.document_text = full_text
        return full_text

    def get_document_id(self):
        return self.document_id

    def get_persons(self):
        pattern = re.compile(r"\b\d{8}-\d{4}\s+[A-Za-z]+(?:\s+[A-Za-z]+)*,\s*[A-Za-z]+(?:\s+[A-Za-z]+)*\.?")
        matches = pattern.findall(self.document_text)
        clean_matches = []
        for person in matches:
            number = "".join([i for i in person if i.isdigit()])
            name = "".join([i for i in person if not i.isdigit()]).replace('- ', ' ').replace('.', '')
            parts = name.split(", ")
            if len(parts) == 2:
                last, first = parts
            else:
                last, first = parts[0], ""
            clean_matches.append({
                "id": number,
                "first_name": first.strip(),
                "last_name": last.strip()
            })
        return clean_matches

    def get_information_codes(self):
        valuecode_match = re.search(r"grundbearbetning:\s*([A-Z]\d)", self.document_text)
        handlingcode_match = re.search(r"användningsvillkor:\s*([A-Z]\d)", self.document_text)
        valuecode = valuecode_match.group(1) if valuecode_match else ""
        handlingcode = handlingcode_match.group(1) if handlingcode_match else ""
        return [valuecode, handlingcode]

    def get_date(self):
        patterns = [
            r"inhämtad\s+(\d{4}-\d{2}-\d{2})",
            r"datum:\s+(\d{4}-\d{2}-\d{2})",
            r"datum\s+(\d{4}-\d{2}-\d{2})"
        ]
        date = None
        for pattern in patterns:
            match = re.search(pattern, self.document_text.lower())
            if match:
                date = match.group(1)
                break
        return date

# =============================================================================
# Data Model: GraphManager and Node/Edge Dataclasses
# =============================================================================

@dataclass
class BaseNode:
    id: str
    metadata: Dict[str, Any] = field(default_factory=dict)
    edges: List["Edge"] = field(default_factory=list)

    def __post_init__(self):
        self.id = str(self.id)
        self.type = self.__class__.__name__

@dataclass(kw_only=True)
class PersonNode(BaseNode):
    first_name: str
    last_name: str
    is_unknown: bool = False

@dataclass(kw_only=True)
class ActivityNode(BaseNode):
    description: str
    activity_date: str

@dataclass(kw_only=True)
class ObjectNode(BaseNode):
    description: str

@dataclass
class Edge:
    node_a_id: str
    node_b_id: str
    description: str

class GraphManager:
    def __init__(self):
        self.nodes: Dict[str, BaseNode] = {}
        self.edges: List[Edge] = []
        self.current_pdf_metadata: Dict[str, Any] = {}
        self.source_text: str = ""

    def add_node(self, node: BaseNode) -> None:
        if node.id in self.nodes:
            return
        self.nodes[node.id] = node

    def add_edge(self, node_a_id: str, node_b_id: str, description: str = "") -> Edge:
        if node_a_id not in self.nodes or node_b_id not in self.nodes:
            raise ValueError("Both nodes must exist to create an edge.")
        for existing in self.edges:
            if (existing.node_a_id == node_a_id and existing.node_b_id == node_b_id and
                existing.description == description):
                return existing
        edge = Edge(node_a_id=node_a_id, node_b_id=node_b_id, description=description)
        self.edges.append(edge)
        self.nodes[node_a_id].edges.append(edge)
        self.nodes[node_b_id].edges.append(edge)
        return edge

    def create_person(self, id: str, first_name: str, last_name: str, metadata: Optional[Dict[str, Any]] = None, is_unknown: bool = False) -> PersonNode:
        person = PersonNode(
            id=id,
            first_name=first_name,
            last_name=last_name,
            metadata=metadata or {},
            is_unknown=is_unknown
        )
        self.add_node(person)
        return person

    def create_activity(self, id: str, description: str, activity_date: str, metadata: Optional[Dict[str, Any]] = None) -> ActivityNode:
        if not id.strip():
            id = str(uuid.uuid4())
        if metadata is None and self.current_pdf_metadata:
            metadata = self.current_pdf_metadata
        activity = ActivityNode(
            id=id,
            description=description,
            activity_date=activity_date,
            metadata=metadata or {}
        )
        self.add_node(activity)
        return activity

    def create_object(self, description: str, metadata: Optional[Dict[str, Any]] = None) -> ObjectNode:
        object_id = str(uuid.uuid4())
        object_node = ObjectNode(
            id=object_id,
            description=description,
            metadata=metadata or {}
        )
        self.add_node(object_node)
        return object_node

    def to_dict(self):
        nodes_list = []
        for node in self.nodes.values():
            node_dict = asdict(node)
            if "edges" in node_dict:
                del node_dict["edges"]
            node_dict["type"] = node.type
            nodes_list.append(node_dict)
        return {
            "nodes": nodes_list,
            "edges": [asdict(edge) for edge in self.edges]
        }

    def load_from_dict(self, data: Dict[str, Any]):
        self.nodes.clear()
        self.edges.clear()
        type_map = {
            "PersonNode": PersonNode,
            "ActivityNode": ActivityNode,
            "ObjectNode": ObjectNode
        }
        for node_data in data.get("nodes", []):
            node_data.pop("edges", None)
            node_type = node_data.pop("type", "BaseNode")
            cls = type_map.get(node_type, BaseNode)
            valid_keys = {f.name for f in fields(cls)}
            filtered_data = {k: v for k, v in node_data.items() if k in valid_keys}
            node = cls(**filtered_data)
            self.nodes[node.id] = node
        for edge_data in data.get("edges", []):
            edge = Edge(**edge_data)
            self.edges.append(edge)
            if edge.node_a_id in self.nodes:
                self.nodes[edge.node_a_id].edges.append(edge)
            if edge.node_b_id in self.nodes:
                self.nodes[edge.node_b_id].edges.append(edge)

# =============================================================================
# AnnotationHighlighter: Custom highlighter to format selected text.
# =============================================================================

class AnnotationHighlighter(QSyntaxHighlighter):
    def __init__(self, parent):
        super().__init__(parent)
        self.selection_start = None
        self.selection_end = None
        self.default_font_size = parent.defaultFont().pointSize()
    
    def highlightBlock(self, text):
        if self.selection_start is None or self.selection_end is None:
            return
        block_start = self.currentBlock().position()
        block_end = block_start + len(text)
        sel_start = max(self.selection_start, block_start)
        sel_end = min(self.selection_end, block_end)
        if sel_start < sel_end:
            fmt = QTextCharFormat()
            fmt.setBackground(QColor("#ADD8E6"))
            fmt.setFontWeight(QFont.Bold)
            fmt.setFontPointSize(self.default_font_size + 2)
            self.setFormat(sel_start - block_start, sel_end - sel_start, fmt)

# =============================================================================
# AnnotationTextEdit: Subclass of QTextEdit that uses AnnotationHighlighter.
# =============================================================================

class AnnotationTextEdit(QTextEdit):
    def __init__(self, *args, **kwargs):
        super(AnnotationTextEdit, self).__init__(*args, **kwargs)
        self.setReadOnly(True)
        self.highlighter = AnnotationHighlighter(self.document())
        self.selectionChanged.connect(self.updateHighlighting)
    
    def updateHighlighting(self):
        cursor = self.textCursor()
        if cursor.hasSelection():
            self.highlighter.selection_start = cursor.selectionStart()
            self.highlighter.selection_end = cursor.selectionEnd()
        else:
            self.highlighter.selection_start = None
            self.highlighter.selection_end = None
        QTimer.singleShot(0, self.highlighter.rehighlight)

# =============================================================================
# Visualization: NodeItem and EdgeItem for QGraphicsScene
# =============================================================================

NODE_RADIUS = 30

class NodeItem(QGraphicsEllipseItem):
    def __init__(self, node: BaseNode, graph_widget):
        super().__init__(-NODE_RADIUS, -NODE_RADIUS, NODE_RADIUS * 2, NODE_RADIUS * 2)
        self.node = node
        self.graph_widget = graph_widget
        self.setFlags(QGraphicsItem.ItemIsSelectable | QGraphicsItem.ItemIsMovable | QGraphicsItem.ItemSendsScenePositionChanges)
        self.setToolTip(f"{node.type}: {node.id}")
        self.setZValue(1)

        if node.type == "ActivityNode":
            fill_color = QColor("red")
            label = node.description
        elif node.type == "PersonNode":
            if getattr(node, "is_unknown", False):
                fill_color = QColor("gray")
                label = f"{node.first_name} {node.last_name} (Unknown)"
            else:
                fill_color = QColor("blue")
                label = f"{node.first_name} {node.last_name}"
        elif node.type == "ObjectNode":
            fill_color = QColor("green")
            label = node.description
        else:
            fill_color = QColor("gray")
            label = node.id

        outline_color = fill_color.darker(150)
        self.setBrush(QBrush(fill_color))
        self.setPen(QPen(outline_color, 2))

        self.text_item = QGraphicsTextItem(label, self)
        self.text_item.setDefaultTextColor(Qt.white)
        font = QFont()
        font.setPointSize(8)
        self.text_item.setFont(font)
        rect = self.text_item.boundingRect()
        self.text_item.setPos(-rect.width() / 2, -rect.height() / 2)

    def get_center(self) -> QPointF:
        return self.scenePos()

    def mouseDoubleClickEvent(self, event):
        if self.node.type == "PersonNode":
            dialog = QDialog()
            dialog.setWindowTitle("Edit Person")
            form = QFormLayout(dialog)
            first_edit = QLineEdit(self.node.first_name)
            last_edit = QLineEdit(self.node.last_name)
            form.addRow("First Name:", first_edit)
            form.addRow("Last Name:", last_edit)
            btns = QHBoxLayout()
            ok_btn = QPushButton("OK")
            cancel_btn = QPushButton("Cancel")
            btns.addWidget(ok_btn)
            btns.addWidget(cancel_btn)
            form.addRow(btns)
            ok_btn.clicked.connect(dialog.accept)
            cancel_btn.clicked.connect(dialog.reject)
            if dialog.exec_() == QDialog.Accepted:
                self.graph_widget.record_state()
                self.node.first_name = first_edit.text().strip()
                self.node.last_name = last_edit.text().strip()
                self.text_item.setPlainText(f"{self.node.first_name} {self.node.last_name}" +
                                            (" (Unknown)" if getattr(self.node, "is_unknown", False) else ""))
        elif self.node.type == "ActivityNode":
            dialog = QDialog()
            dialog.setWindowTitle("Edit Activity")
            form = QFormLayout(dialog)
            desc_edit = QLineEdit(self.node.description)
            date_edit = QLineEdit(self.node.activity_date)
            form.addRow("Description:", desc_edit)
            form.addRow("Date (YYYY-MM-DD):", date_edit)
            btns = QHBoxLayout()
            ok_btn = QPushButton("OK")
            cancel_btn = QPushButton("Cancel")
            btns.addWidget(ok_btn)
            btns.addWidget(cancel_btn)
            form.addRow(btns)
            ok_btn.clicked.connect(dialog.accept)
            cancel_btn.clicked.connect(dialog.reject)
            if dialog.exec_() == QDialog.Accepted:
                self.graph_widget.record_state()
                self.node.description = desc_edit.text().strip()
                self.node.activity_date = date_edit.text().strip()
                self.text_item.setPlainText(self.node.description)
        elif self.node.type == "ObjectNode":
            dialog = QDialog()
            dialog.setWindowTitle("Edit Object")
            form = QFormLayout(dialog)
            desc_edit = QLineEdit(self.node.description)
            form.addRow("Description:", desc_edit)
            btns = QHBoxLayout()
            ok_btn = QPushButton("OK")
            cancel_btn = QPushButton("Cancel")
            btns.addWidget(ok_btn)
            btns.addWidget(cancel_btn)
            form.addRow(btns)
            ok_btn.clicked.connect(dialog.accept)
            cancel_btn.clicked.connect(dialog.reject)
            if dialog.exec_() == QDialog.Accepted:
                self.graph_widget.record_state()
                self.node.description = desc_edit.text().strip()
                self.text_item.setPlainText(self.node.description)
        super().mouseDoubleClickEvent(event)

    def mousePressEvent(self, event):
        self._oldPos = self.pos()
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        super().mouseReleaseEvent(event)
        if hasattr(self, '_oldPos') and self.pos() != self._oldPos:
            self.graph_widget.record_state()
        else:
            self.graph_widget.record_state()

    def itemChange(self, change, value):
        if change == QGraphicsItem.ItemPositionHasChanged and self.scene():
            for item in self.scene().items():
                if isinstance(item, EdgeItem):
                    item.update_path()
        return super().itemChange(change, value)

class EdgeItem(QGraphicsPathItem):
    def __init__(self, source: NodeItem, dest: NodeItem, description: str = "", offset_index: int = 0):
        super().__init__()
        self.source = source
        self.dest = dest
        self.description = description
        self.offset_index = offset_index
        self.setZValue(0)
        self.pen = QPen(QColor("white"), 1)
        self.setPen(self.pen)
        self.setBrush(QBrush(Qt.NoBrush))
        self.text_item = QGraphicsTextItem(self.description, self)
        self.text_item.setDefaultTextColor(QColor("white"))
        font = QFont()
        font.setPointSize(8)
        self.text_item.setFont(font)
        self.update_path()

    def update_path(self):
        src_center = self.source.get_center()
        dst_center = self.dest.get_center()
        v = dst_center - src_center
        length = math.hypot(v.x(), v.y())
        if length == 0:
            start = src_center
            end = dst_center
        else:
            start = src_center + v / length * NODE_RADIUS
            end = dst_center - v / length * NODE_RADIUS
        path = QPainterPath()
        path.moveTo(start)
        if length == 0:
            perp = QPointF(0, 0)
        else:
            perp_angle = math.atan2(v.y(), v.x()) + math.pi/2
            base_offset = length / 8
            extra_offset = self.compute_extra_offset(self.offset_index)
            total_offset = base_offset + extra_offset
            perp = QPointF(total_offset * math.cos(perp_angle), total_offset * math.sin(perp_angle))
        ctrl1 = start + perp
        ctrl2 = end + perp
        path.cubicTo(ctrl1, ctrl2, end)
        self.setPath(path)
        mid = path.pointAtPercent(0.5)
        self.text_item.setPos(mid.x(), mid.y())

    def compute_extra_offset(self, index: int) -> float:
        if index == 0:
            return 0
        sign = 1 if index % 2 == 1 else -1
        multiplier = (index + 1) // 2
        return sign * multiplier * 10

    def mouseDoubleClickEvent(self, event):
        new_desc, ok = QInputDialog.getText(None, "Edit Edge", "Enter new edge description:", text=self.description)
        if ok:
            self.source.graph_widget.record_state()
            self.description = new_desc.strip()
            self.text_item.setPlainText(self.description)
            for edge in self.source.node.edges:
                if ({edge.node_a_id, edge.node_b_id} == {self.source.node.id, self.dest.node.id}):
                    edge.description = self.description
        super().mouseDoubleClickEvent(event)

# =============================================================================
# GraphWidget: Displays and edits the graph.
# =============================================================================

class GraphWidget(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setRenderHint(QPainter.Antialiasing)
        self.setScene(QGraphicsScene(self))
        self.setDragMode(QGraphicsView.RubberBandDrag)
        self.node_items: Dict[str, NodeItem] = {}
        self.edge_items: List[EdgeItem] = []
        self.current_graph: Optional[GraphManager] = None
        self.undo_stack = []
        self.redo_stack = []
        self.on_refresh_callback = None
        self.current_hover_node: Optional[NodeItem] = None
        self.edge_creation_sources: List[NodeItem] = []
        self.temp_edge_items: List[QGraphicsPathItem] = []
        self.layoutAnimations = []
        self._pan_start = None

    def record_state(self):
        if self.current_graph is None:
            return
        state = {
            'graph': copy.deepcopy(self.current_graph),
            'positions': {node_id: (item.pos().x(), item.pos().y()) for node_id, item in self.node_items.items()}
        }
        self.undo_stack.append(state)
        self.redo_stack.clear()
        if self.on_refresh_callback:
            self.on_refresh_callback()

    def undo(self):
        if not self.undo_stack:
            return
        current_state = {
            'graph': copy.deepcopy(self.current_graph),
            'positions': {node_id: (item.pos().x(), item.pos().y()) for node_id, item in self.node_items.items()}
        }
        self.redo_stack.append(current_state)
        state = self.undo_stack.pop()
        self.current_graph.nodes = state['graph'].nodes
        self.current_graph.edges = state['graph'].edges
        self.refresh(self.current_graph)
        for node_id, pos in state['positions'].items():
            if node_id in self.node_items:
                self.node_items[node_id].setPos(QPointF(pos[0], pos[1]))
        self.scene().update()

    def redo(self):
        if not self.redo_stack:
            return
        current_state = {
            'graph': copy.deepcopy(self.current_graph),
            'positions': {node_id: (item.pos().x(), item.pos().y()) for node_id, item in self.node_items.items()}
        }
        self.undo_stack.append(current_state)
        state = self.redo_stack.pop()
        self.current_graph.nodes = state['graph'].nodes
        self.current_graph.edges = state['graph'].edges
        self.refresh(self.current_graph)
        for node_id, pos in state['positions'].items():
            if node_id in self.node_items:
                self.node_items[node_id].setPos(QPointF(pos[0], pos[1]))
        self.scene().update()

    def clear(self):
        self.scene().clear()
        self.node_items.clear()
        self.edge_items.clear()

    def add_node_item(self, node: BaseNode, position: Optional[QPointF] = None):
        item = NodeItem(node, self)
        item.setFlag(QGraphicsItem.ItemIsMovable, True)
        if position is None:
            position = QPointF(0, 0)
        item.setPos(position)
        self.scene().addItem(item)
        self.node_items[node.id] = item

    def add_edge_item(self, edge: Edge):
        source_item = self.node_items.get(edge.node_a_id)
        dest_item = self.node_items.get(edge.node_b_id)
        if source_item and dest_item:
            count = 0
            for existing_edge in self.edge_items:
                if ({existing_edge.source.node.id, existing_edge.dest.node.id} ==
                    {source_item.node.id, dest_item.node.id} and
                    existing_edge.description == edge.description):
                    count += 1
            edge_item = EdgeItem(source_item, dest_item, edge.description, offset_index=count)
            self.scene().addItem(edge_item)
            self.edge_items.append(edge_item)

    def refresh(self, graph: 'GraphManager'):
        self.current_graph = graph
        saved_positions = {node_id: item.pos() for node_id, item in self.node_items.items()}
        self.clear()
        num_nodes = len(graph.nodes)
        for i, node in enumerate(graph.nodes.values()):
            if node.id in saved_positions:
                pos = saved_positions[node.id]
            else:
                radius = 200
                angle_step = 360 / max(num_nodes, 1)
                angle = math.radians(i * angle_step)
                pos = QPointF(radius * math.cos(angle), radius * math.sin(angle))
            self.add_node_item(node, pos)
        for edge in graph.edges:
            self.add_edge_item(edge)
        self.scene().update()
        if self.on_refresh_callback:
            self.on_refresh_callback()

    def organize_layout(self):
        if not self.node_items or self.current_graph is None:
            return
        width = self.viewport().width()
        height = self.viewport().height()
        area = width * height
        n = len(self.node_items)
        k = math.sqrt(area / n)
        iterations = 50
        dt = 0.1
        positions = {node_id: self.node_items[node_id].pos() for node_id in self.node_items}
        for _ in range(iterations):
            forces = {node_id: QPointF(0, 0) for node_id in self.node_items}
            for id1, pos1 in positions.items():
                for id2, pos2 in positions.items():
                    if id1 == id2:
                        continue
                    delta = pos1 - pos2
                    distance = math.hypot(delta.x(), delta.y())
                    if distance < 1:
                        distance = 1
                    force_magnitude = (k * k) / distance
                    forces[id1] += QPointF(delta.x() / distance * force_magnitude,
                                             delta.y() / distance * force_magnitude)
            for edge in self.current_graph.edges:
                if edge.node_a_id in positions and edge.node_b_id in positions:
                    pos_a = positions[edge.node_a_id]
                    pos_b = positions[edge.node_b_id]
                    delta = pos_a - pos_b
                    distance = math.hypot(delta.x(), delta.y())
                    if distance < 1:
                        distance = 1
                    force_magnitude = (distance * distance) / k
                    force = QPointF(delta.x() / distance * force_magnitude,
                                    delta.y() / distance * force_magnitude)
                    forces[edge.node_a_id] -= force
                    forces[edge.node_b_id] += force
            for node_id in positions:
                disp = forces[node_id]
                max_disp = 20
                disp_length = math.hypot(disp.x(), disp.y())
                if disp_length > max_disp:
                    disp = QPointF(disp.x() / disp_length * max_disp, disp.y() / disp_length * max_disp)
                positions[node_id] += QPointF(disp.x() * dt, disp.y() * dt)
        self.layoutAnimations = []
        duration = 500
        for node_id, new_pos in positions.items():
            node_item = self.node_items[node_id]
            animation = QVariantAnimation(self)
            animation.setDuration(duration)
            animation.setStartValue(node_item.pos())
            animation.setEndValue(new_pos)
            animation.valueChanged.connect(lambda value, item=node_item: item.setPos(value))
            animation.start()
            self.layoutAnimations.append(animation)
        QTimer.singleShot(duration + 50, lambda: self.layoutAnimations.clear())

    def wheelEvent(self, event):
        zoom_in_factor = 1.15
        zoom_out_factor = 1 / zoom_in_factor
        factor = zoom_in_factor if event.angleDelta().y() > 0 else zoom_out_factor
        self.scale(factor, factor)

    def mousePressEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._pan_start = event.pos()
            self.setCursor(Qt.ClosedHandCursor)
            event.accept()
            return
        pos = self.mapToScene(event.pos())
        item = self.scene().itemAt(pos, self.transform())
        if event.button() == Qt.LeftButton and (event.modifiers() & Qt.ControlModifier):
            selected_items = [it for it in self.scene().selectedItems() if isinstance(it, NodeItem)]
            if selected_items:
                self.edge_creation_sources = selected_items
            elif isinstance(item, NodeItem):
                self.edge_creation_sources = [item]
            else:
                self.edge_creation_sources = []
            if self.edge_creation_sources:
                for src in self.edge_creation_sources:
                    src.setScale(1.2)
                self.temp_edge_items = []
                for src in self.edge_creation_sources:
                    temp_edge = QGraphicsPathItem()
                    dash_pen = QPen(QColor("white"), 1, Qt.DashLine)
                    temp_edge.setPen(dash_pen)
                    temp_edge.setBrush(QBrush(Qt.NoBrush))
                    temp_edge.setAcceptedMouseButtons(Qt.NoButton)
                    self.scene().addItem(temp_edge)
                    self.temp_edge_items.append(temp_edge)
                event.accept()
                return
        super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_start is not None:
            delta = event.pos() - self._pan_start
            self._pan_start = event.pos()
            self.horizontalScrollBar().setValue(self.horizontalScrollBar().value() - delta.x())
            self.verticalScrollBar().setValue(self.verticalScrollBar().value() - delta.y())
            event.accept()
            return
        if self.edge_creation_sources and self.temp_edge_items:
            pos = self.mapToScene(event.pos())
            for src, temp_edge in zip(self.edge_creation_sources, self.temp_edge_items):
                start = src.get_center()
                path = QPainterPath()
                path.moveTo(start)
                ctrl = (start + pos) * 0.5
                path.quadTo(ctrl, pos)
                temp_edge.setPath(path)
            hover_item = self.scene().itemAt(pos, self.transform())
            if hover_item is not None and isinstance(hover_item, NodeItem) and hover_item not in self.edge_creation_sources:
                if self.current_hover_node is not hover_item:
                    if self.current_hover_node is not None:
                        self.current_hover_node.setScale(1.0)
                    self.current_hover_node = hover_item
                    self.current_hover_node.setScale(1.2)
            else:
                if self.current_hover_node is not None:
                    self.current_hover_node.setScale(1.0)
                    self.current_hover_node = None
            event.accept()
            return
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MiddleButton:
            self._pan_start = None
            self.setCursor(Qt.ArrowCursor)
            event.accept()
            return
        if self.edge_creation_sources and self.temp_edge_items:
            pos = self.mapToScene(event.pos())
            dest_item = self.scene().itemAt(pos, self.transform())
            for temp_edge in self.temp_edge_items:
                self.scene().removeItem(temp_edge)
            self.temp_edge_items = []
            if isinstance(dest_item, NodeItem) and dest_item not in self.edge_creation_sources:
                new_desc, ok = QInputDialog.getText(self, "Edge Description", "Enter edge description:")
                if ok:
                    self.record_state()
                    for src in self.edge_creation_sources:
                        try:
                            self.current_graph.add_edge(src.node.id, dest_item.node.id, new_desc.strip())
                        except ValueError as e:
                            print(e)
            for src in self.edge_creation_sources:
                src.setScale(1.0)
            self.edge_creation_sources = []
            if self.current_hover_node:
                self.current_hover_node.setScale(1.0)
                self.current_hover_node = None
            self.refresh(self.current_graph)
            event.accept()
            return
        super().mouseReleaseEvent(event)

    def filter_nodes(self, search_text: str):
        search_text = search_text.lower()
        for node_id, node_item in self.node_items.items():
            node = node_item.node
            match = False
            if node.type == "PersonNode":
                full_name = f"{node.first_name} {node.last_name}".lower()
                if search_text in full_name or search_text in node.id.lower():
                    match = True
            elif node.type in ["ActivityNode", "ObjectNode"]:
                if search_text in node.description.lower() or search_text in node.id.lower():
                    match = True
            else:
                if search_text in node.id.lower():
                    match = True
            if match or search_text == "":
                node_item.show()
            else:
                node_item.hide()
        for edge_item in self.edge_items:
            if edge_item.source.isVisible() and edge_item.dest.isVisible():
                edge_item.show()
            else:
                edge_item.hide()

    def clear_filter(self):
        for node_item in self.node_items.values():
            node_item.show()
        for edge_item in self.edge_items:
            edge_item.show()

# =============================================================================
# Form Widgets for Data Entry (with keyboard shortcuts)
# =============================================================================

class ActivityFormWidget(QWidget):
    def __init__(self, graph_manager: GraphManager, graph_widget: GraphWidget, parent=None):
        super().__init__(parent)
        self.graph_manager = graph_manager
        self.graph_widget = graph_widget
        layout = QFormLayout(self)
        self.id_edit = QLineEdit()
        self.desc_edit = QLineEdit()
        self.date_edit = QLineEdit()
        layout.addRow(QLabel("Activity ID:"), self.id_edit)
        layout.addRow(QLabel("Description:"), self.desc_edit)
        layout.addRow(QLabel("Date (YYYY-MM-DD):"), self.date_edit)
        self.submit_btn = QPushButton("Add Activity")
        self.submit_btn.clicked.connect(self.add_activity)
        layout.addRow(self.submit_btn)
        self.date_edit.returnPressed.connect(self.add_activity)

    def add_activity(self):
        act_id = self.id_edit.text().strip()
        if not act_id:
            act_id = str(uuid.uuid4())
        desc = self.desc_edit.text().strip()
        date = self.date_edit.text().strip()
        try:
            self.graph_widget.record_state()
            self.graph_manager.create_activity(act_id, desc, date)
            self.graph_widget.refresh(self.graph_manager)
            self.id_edit.clear()
            self.desc_edit.clear()
        except ValueError as e:
            print(e)

class PersonFormWidget(QWidget):
    def __init__(self, graph_manager: GraphManager, graph_widget: GraphWidget, parent=None):
        super().__init__(parent)
        self.graph_manager = graph_manager
        self.graph_widget = graph_widget
        layout = QFormLayout(self)
        self.id_edit = QLineEdit()
        self.first_edit = QLineEdit()
        self.last_edit = QLineEdit()
        layout.addRow(QLabel("Person ID:"), self.id_edit)
        layout.addRow(QLabel("First Name:"), self.first_edit)
        layout.addRow(QLabel("Last Name:"), self.last_edit)
        self.unknown_checkbox = QCheckBox("Unknown Person")
        self.unknown_checkbox.toggled.connect(self.on_unknown_toggled)
        layout.addRow(self.unknown_checkbox)
        self.submit_btn = QPushButton("Add Person")
        self.submit_btn.clicked.connect(self.add_person)
        layout.addRow(self.submit_btn)
        self.last_edit.returnPressed.connect(self.add_person)

    def on_unknown_toggled(self, checked):
        if checked:
            self.first_edit.setText("Unknown")
            self.first_edit.setDisabled(True)
            self.last_edit.setText("Unknown")
            self.last_edit.setDisabled(True)
        else:
            self.first_edit.setText("")
            self.first_edit.setDisabled(False)
            self.last_edit.setText("")
            self.last_edit.setDisabled(False)

    def add_person(self):
        p_id = self.id_edit.text().strip()
        if not p_id:
            p_id = str(uuid.uuid4())
        first = self.first_edit.text().strip()
        last = self.last_edit.text().strip()
        is_unknown = self.unknown_checkbox.isChecked()
        try:
            self.graph_widget.record_state()
            self.graph_manager.create_person(p_id, first, last, is_unknown=is_unknown)
            self.graph_widget.refresh(self.graph_manager)
            self.id_edit.clear()
            self.first_edit.clear()
            self.last_edit.clear()
            self.unknown_checkbox.setChecked(False)
        except ValueError as e:
            print(e)

# =============================================================================
# ObjectFormWidget with Predefined Templates, QLineEdits, and Updated Tab Order
# =============================================================================

class ObjectFormWidget(QWidget):
    def __init__(self, graph_manager: GraphManager, graph_widget: GraphWidget, parent=None):
        super().__init__(parent)
        self.graph_manager = graph_manager
        self.graph_widget = graph_widget
        
        # Use a single QFormLayout so that the "Description:" row and dynamic template rows align.
        self.layout = QVBoxLayout(self)
        self.form_layout = QFormLayout()
        self.layout.addLayout(self.form_layout)
        
        # Updated templates: all fields now use QLineEdit.
        self.templates = {
            "Car": {
                "License Plate": QLineEdit,
                "Model": QLineEdit,
                "Year": QLineEdit,
            },
            "Product": {
                "Quantity": QLineEdit,
                "Price": QLineEdit,
            }
        }
        self.current_template = None
        self.template_widgets = {}
        self.template_widgets_order = []  # to maintain the order of dynamic fields
        
        self.desc_edit = QLineEdit()
        self.form_layout.addRow("Description:", self.desc_edit)
        # Allow pressing Enter in the description field to add the object
        self.desc_edit.returnPressed.connect(self.add_object)
        self.desc_edit.textChanged.connect(self.check_template)
        
        self.submit_btn = QPushButton("Add Object")
        self.submit_btn.clicked.connect(self.add_object)
        self.layout.addWidget(self.submit_btn)
        
        # Set initial tab order: description -> submit button
        QWidget.setTabOrder(self.desc_edit, self.submit_btn)
    
    def check_template(self, text):
        text = text.strip()
        if text in self.templates:
            if self.current_template != text:
                self.clear_template_fields()
                self.current_template = text
                self.template_widgets_order = []
                for label, widget_class in self.templates[text].items():
                    widget = widget_class()
                    widget.returnPressed.connect(self.add_object)
                    self.form_layout.addRow(label + ":", widget)
                    self.template_widgets[label] = widget
                    self.template_widgets_order.append(widget)
                self.update_tab_order()
        else:
            if self.current_template is not None:
                self.clear_template_fields()
                self.current_template = None
                self.update_tab_order()
    
    def update_tab_order(self):
        # Set tab order: description -> dynamic fields (if any) -> submit button.
        if self.template_widgets_order:
            previous = self.desc_edit
            for widget in self.template_widgets_order:
                QWidget.setTabOrder(previous, widget)
                previous = widget
            QWidget.setTabOrder(previous, self.submit_btn)
        else:
            QWidget.setTabOrder(self.desc_edit, self.submit_btn)
    
    def clear_template_fields(self):
        for label, widget in list(self.template_widgets.items()):
            self.remove_row_containing_widget(widget)
        self.template_widgets = {}
        self.template_widgets_order = []
    
    def remove_row_containing_widget(self, widget):
        row_count = self.form_layout.rowCount()
        for row in range(row_count):
            field_item = self.form_layout.itemAt(row, QFormLayout.FieldRole)
            if field_item and field_item.widget() == widget:
                self.form_layout.removeRow(row)
                break
    
    def add_object(self):
        desc = self.desc_edit.text().strip()
        metadata = {}
        if self.current_template:
            fields_data = {}
            for label, widget in self.template_widgets.items():
                fields_data[label] = widget.text().strip()
            metadata = {"template": self.current_template, "fields": fields_data}
        try:
            self.graph_widget.record_state()
            self.graph_manager.create_object(desc, metadata)
            self.graph_widget.refresh(self.graph_manager)
            self.desc_edit.clear()
            self.clear_template_fields()
            self.current_template = None
            self.update_tab_order()
        except ValueError as e:
            print(e)

class EdgeFormWidget(QWidget):
    def __init__(self, graph_manager: GraphManager, graph_widget: GraphWidget, parent=None):
        super().__init__(parent)
        self.graph_manager = graph_manager
        self.graph_widget = graph_widget
        layout = QFormLayout(self)
        self.source_combo = QComboBox()
        self.dest_combo = QComboBox()
        self.source_combo.setEditable(False)
        self.dest_combo.setEditable(False)
        self.desc_edit = QLineEdit()
        layout.addRow(QLabel("Source Node:"), self.source_combo)
        layout.addRow(QLabel("Destination Node:"), self.dest_combo)
        layout.addRow(QLabel("Description:"), self.desc_edit)
        self.submit_btn = QPushButton("Add Edge")
        self.submit_btn.clicked.connect(self.add_edge)
        layout.addRow(self.submit_btn)
        self.refresh_combo()
        self.desc_edit.returnPressed.connect(self.add_edge)

    def refresh_combo(self):
        self.source_combo.clear()
        self.dest_combo.clear()
        for node in self.graph_manager.nodes.values():
            if node.type == "PersonNode":
                name = f"{node.first_name} {node.last_name}".strip()
                display = f"{name} ({node.id})" if name else node.id
            elif node.type in ["ActivityNode", "ObjectNode"]:
                display = f"{node.description} ({node.id})" if node.description else node.id
            else:
                display = node.id
            self.source_combo.addItem(display, node.id)
            self.dest_combo.addItem(display, node.id)

    def add_edge(self):
        source_id = self.source_combo.currentData() or self.source_combo.currentText().strip()
        dest_id = self.dest_combo.currentData() or self.dest_combo.currentText().strip()
        desc = self.desc_edit.text().strip()
        if not source_id or not dest_id:
            print("Please select valid source and destination nodes.")
            return
        if source_id == dest_id:
            print("Source and destination must be different.")
            return
        if source_id not in self.graph_manager.nodes or dest_id not in self.graph_manager.nodes:
            print("One or both node IDs not found in the graph.")
            return
        try:
            self.graph_widget.record_state()
            self.graph_manager.add_edge(source_id, dest_id, desc)
            self.graph_widget.refresh(self.graph_manager)
            self.desc_edit.clear()
        except ValueError as e:
            print(e)

# =============================================================================
# DataViewWidget: Displays the combined JSON data.
# =============================================================================

class DataViewWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.graph_manager = None
        layout = QVBoxLayout(self)
        self.data_edit = QPlainTextEdit()
        self.data_edit.setReadOnly(True)
        self.refresh_btn = QPushButton("Refresh Data")
        self.refresh_btn.clicked.connect(self.refresh_data)
        layout.addWidget(self.data_edit)
        layout.addWidget(self.refresh_btn)
        self.refresh_data()

    def set_graph_manager(self, gm: 'GraphManager'):
        self.graph_manager = gm

    def refresh_data(self):
        if self.graph_manager:
            data = self.graph_manager.to_dict()
            self.data_edit.setPlainText(json.dumps(data, indent=4))
        else:
            self.data_edit.setPlainText("No data available.")

# =============================================================================
# EditorWidget: Combines the form widgets and the graph view.
# =============================================================================

class EditorWidget(QWidget):
    def __init__(self, active_graph_manager: GraphManager, graph_widget: GraphWidget, parent=None):
        super().__init__(parent)
        self.active_graph_manager = active_graph_manager
        self.graph_widget = graph_widget

        activity_group = QGroupBox("Activity")
        activity_group.setStyleSheet(
            "QGroupBox { background-color: rgba(255, 0, 0, 50); border: 1px solid gray; border-radius: 5px; margin: 5px; padding: 5px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"
        )
        activity_layout = QVBoxLayout()
        self.activity_form = ActivityFormWidget(active_graph_manager, graph_widget)
        activity_layout.addWidget(self.activity_form)
        activity_group.setLayout(activity_layout)

        person_group = QGroupBox("Person")
        person_group.setStyleSheet(
            "QGroupBox { background-color: rgba(0, 0, 255, 50); border: 1px solid gray; border-radius: 5px; margin: 5px; padding: 5px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"
        )
        person_layout = QVBoxLayout()
        self.person_form = PersonFormWidget(active_graph_manager, graph_widget)
        person_layout.addWidget(self.person_form)
        person_group.setLayout(person_layout)

        object_group = QGroupBox("Object")
        object_group.setStyleSheet(
            "QGroupBox { background-color: rgba(0, 128, 0, 50); border: 1px solid gray; border-radius: 5px; margin: 5px; padding: 5px; }"
            "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 3px 0 3px; }"
        )
        object_layout = QVBoxLayout()
        self.object_form = ObjectFormWidget(active_graph_manager, graph_widget)
        object_layout.addWidget(self.object_form)
        object_group.setLayout(object_layout)

        top_layout = QHBoxLayout()
        top_layout.addWidget(activity_group)
        top_layout.addWidget(person_group)
        top_layout.addWidget(object_group)

        edge_layout = QHBoxLayout()
        self.edge_form = EdgeFormWidget(active_graph_manager, graph_widget)
        edge_layout.addWidget(self.edge_form)

        main_layout = QVBoxLayout()
        main_layout.addLayout(top_layout)
        main_layout.addLayout(edge_layout)
        main_layout.addWidget(self.graph_widget)
        self.setLayout(main_layout)

        self.graph_widget.on_refresh_callback = lambda: self.update_refresh_callback()

    def update_refresh_callback(self):
        self.edge_form.refresh_combo()
        main_window = self.window()
        if main_window and hasattr(main_window, "update_data_view"):
            main_window.update_data_view()

# =============================================================================
# MainWindow: Manages sessions, session selector, export, and annotation mode.
# =============================================================================

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Social Network Graph Editor")
        self.sessions: Dict[str, GraphManager] = {}
        self.active_session_key: Optional[str] = None
        self.active_graph_manager = GraphManager()

        self.graph_widget = GraphWidget()
        self.graph_widget.refresh(self.active_graph_manager)

        self.data_view_widget = DataViewWidget()

        self.editor_widget = EditorWidget(self.active_graph_manager, self.graph_widget)

        self.tab_widget = QTabWidget()
        self.tab_widget.addTab(self.editor_widget, "Editor")
        self.tab_widget.addTab(self.data_view_widget, "Data")
        self.setCentralWidget(self.tab_widget)

        self.create_toolbar()
        self.create_annotation_dock()
        self.create_global_shortcuts()

    def create_toolbar(self):
        toolbar = QToolBar("Main Toolbar", self)
        self.addToolBar(toolbar)

        undo_action = QAction("Undo", self)
        undo_action.triggered.connect(self.graph_widget.undo)
        undo_action.setShortcut("Ctrl+Z")
        toolbar.addAction(undo_action)

        redo_action = QAction("Redo", self)
        redo_action.triggered.connect(self.graph_widget.redo)
        redo_action.setShortcut("Ctrl+Y")
        toolbar.addAction(redo_action)

        self.search_edit = QLineEdit()
        self.search_edit.setPlaceholderText("Search nodes...")
        self.search_edit.textChanged.connect(lambda text: self.graph_widget.filter_nodes(text))
        toolbar.addWidget(self.search_edit)

        clear_filter_action = QAction("Clear Filter", self)
        clear_filter_action.triggered.connect(lambda: (self.graph_widget.clear_filter(), self.search_edit.clear()))
        clear_filter_action.setShortcut("Ctrl+L")
        toolbar.addAction(clear_filter_action)

        save_action = QAction("Save JSON", self)
        save_action.triggered.connect(self.save_json)
        save_action.setShortcut("Ctrl+S")
        load_action = QAction("Load JSON", self)
        load_action.triggered.connect(self.load_json)
        load_action.setShortcut("Ctrl+O")
        toolbar.addAction(save_action)
        toolbar.addAction(load_action)

        load_pdf_action = QAction("Load PDF Document", self)
        load_pdf_action.triggered.connect(self.load_pdf_document)
        load_pdf_action.setShortcut("Ctrl+P")
        toolbar.addAction(load_pdf_action)

        layout_action = QAction("Force Layout", self)
        layout_action.triggered.connect(self.graph_widget.organize_layout)
        toolbar.addAction(layout_action)

        self.toggle_annotation_action = QAction("Toggle Annotation", self)
        self.toggle_annotation_action.setCheckable(True)
        self.toggle_annotation_action.triggered.connect(self.toggle_annotation_mode)
        toolbar.addAction(self.toggle_annotation_action)

        self.session_selector = QComboBox()
        self.session_selector.addItem("Combined View", None)
        self.session_selector.currentIndexChanged.connect(self.session_changed)
        toolbar.addWidget(QLabel("Session: "))
        toolbar.addWidget(self.session_selector)

    def create_annotation_dock(self):
        self.annotationDock = QDockWidget("Annotation Mode", self)
        self.annotationDock.setAllowedAreas(Qt.RightDockWidgetArea | Qt.LeftDockWidgetArea)
        dock_widget = QWidget()
        layout = QVBoxLayout(dock_widget)
        self.annotationTextEdit = AnnotationTextEdit()
        self.annotationTextEdit.setReadOnly(True)
        self.annotationTextEdit.setStyleSheet(
            "QTextEdit { background-color: #fdf6e3; color: #333; border: 1px solid #ccc; padding: 5px; }"
        )
        self.attachAnnotationBtn = QPushButton("Attach Annotation")
        self.attachAnnotationBtn.clicked.connect(self.attach_annotation)
        layout.addWidget(self.annotationTextEdit)
        layout.addWidget(self.attachAnnotationBtn)
        self.annotationDock.setWidget(dock_widget)
        self.addDockWidget(Qt.RightDockWidgetArea, self.annotationDock)
        self.annotationDock.hide()

    def toggle_annotation_mode(self, checked):
        if checked:
            self.annotationDock.show()
        else:
            self.annotationDock.hide()

    def create_global_shortcuts(self):
        QShortcut(QKeySequence("Ctrl+F"), self, activated=lambda: self.search_edit.setFocus())
        QShortcut(QKeySequence("Ctrl+E"), self, activated=lambda: self.tab_widget.setCurrentWidget(self.editor_widget))
        QShortcut(QKeySequence("Ctrl+D"), self, activated=lambda: self.tab_widget.setCurrentWidget(self.data_view_widget))

    def load_pdf_document(self):
        pdf_filename, _ = QFileDialog.getOpenFileName(self, "Load PDF Document", "", "PDF Files (*.pdf)")
        if not pdf_filename:
            return
        doc_handler = DocumentHandler(pdf_filename)
        persons = doc_handler.get_persons()
        doc_id = doc_handler.get_document_id()
        date = doc_handler.get_date()
        info_codes = doc_handler.get_information_codes()
        metadata = {
            "document_id": doc_id,
            "date": date,
            "valuecode": info_codes[0],
            "handlingcode": info_codes[1]
        }
        new_session = GraphManager()
        new_session.current_pdf_metadata = metadata
        new_session.source_text = doc_handler.document_text
        for person in persons:
            new_session.create_person(person["id"], person["first_name"], person["last_name"])
        session_key = doc_id
        self.sessions[session_key] = new_session
        self.session_selector.addItem(f"Session: {doc_id}", session_key)
        self.session_selector.setCurrentIndex(self.session_selector.count() - 1)
        self.active_session_key = session_key
        self.active_graph_manager = new_session
        self.refresh_editor_view()
        print(f"PDF Document '{pdf_filename}' loaded as session '{doc_id}'. Metadata applied: {metadata}")

    def session_changed(self, index):
        session_key = self.session_selector.itemData(index)
        if session_key is None:
            self.active_session_key = None
            combined = self.get_combined_graph()
            self.graph_widget.refresh(combined)
            self.set_forms_enabled(False)
        else:
            self.active_session_key = session_key
            self.active_graph_manager = self.sessions[session_key]
            self.graph_widget.refresh(self.active_graph_manager)
            self.refresh_editor_view()
            self.set_forms_enabled(True)

    def refresh_editor_view(self):
        self.editor_widget.activity_form.graph_manager = self.active_graph_manager
        self.editor_widget.person_form.graph_manager = self.active_graph_manager
        self.editor_widget.object_form.graph_manager = self.active_graph_manager
        self.editor_widget.edge_form.graph_manager = self.active_graph_manager
        if self.active_graph_manager.current_pdf_metadata.get("date"):
            self.editor_widget.activity_form.date_edit.setText(self.active_graph_manager.current_pdf_metadata.get("date"))
        self.graph_widget.refresh(self.active_graph_manager)
        self.annotationTextEdit.setPlainText(self.active_graph_manager.source_text)
        self.update_data_view()

    def set_forms_enabled(self, enabled: bool):
        self.editor_widget.activity_form.setEnabled(enabled)
        self.editor_widget.person_form.setEnabled(enabled)
        self.editor_widget.object_form.setEnabled(enabled)
        self.editor_widget.edge_form.setEnabled(enabled)

    def get_combined_graph(self) -> GraphManager:
        combined = GraphManager()
        edge_set = set()
        for session in self.sessions.values():
            for node in session.nodes.values():
                combined.add_node(node)
            for edge in session.edges:
                key = (edge.node_a_id, edge.node_b_id, edge.description)
                if key not in edge_set:
                    edge_set.add(key)
                    combined.add_edge(edge.node_a_id, edge.node_b_id, edge.description)
        return combined

    def update_data_view(self):
        combined = self.get_combined_graph()
        self.data_view_widget.set_graph_manager(combined)
        self.data_view_widget.refresh_data()

    def attach_annotation(self):
        cursor = self.annotationTextEdit.textCursor()
        selected_text = cursor.selectedText().strip()
        if not selected_text:
            QMessageBox.warning(self, "No Text Selected", "Please highlight some text to attach as annotation.")
            return
        selected_items = [item for item in self.graph_widget.scene().selectedItems() if isinstance(item, NodeItem)]
        if len(selected_items) != 1:
            QMessageBox.warning(self, "Select One Node", "Please select exactly one node to attach the annotation.")
            return
        node_item = selected_items[0]
        if "annotations" not in node_item.node.metadata:
            node_item.node.metadata["annotations"] = []
        node_item.node.metadata["annotations"].append(selected_text)
        self.graph_widget.record_state()
        self.graph_widget.refresh(self.active_graph_manager)
        QMessageBox.information(self, "Annotation Attached", "Annotation attached to the selected node.")

    def save_json(self):
        combined = self.get_combined_graph()
        new_data = combined.to_dict()
        filename, _ = QFileDialog.getSaveFileName(self, "Save Graph", "", "JSON Files (*.json)")
        if filename:
            if os.path.exists(filename):
                with open(filename, "r") as f:
                    existing_data = json.load(f)
            else:
                existing_data = {"nodes": [], "edges": []}
            existing_doc_ids = set()
            for node in existing_data.get("nodes", []):
                md = node.get("metadata", {})
                doc_id = md.get("document_id")
                if doc_id:
                    existing_doc_ids.add(doc_id)
            filtered_new_nodes = []
            for node in new_data.get("nodes", []):
                md = node.get("metadata", {})
                doc_id = md.get("document_id")
                if doc_id:
                    if doc_id not in existing_doc_ids:
                        filtered_new_nodes.append(node)
                        existing_doc_ids.add(doc_id)
                else:
                    if not any(existing_node["id"] == node["id"] for existing_node in existing_data.get("nodes", [])):
                        filtered_new_nodes.append(node)
            merged_nodes = existing_data.get("nodes", []) + filtered_new_nodes
            existing_edges = existing_data.get("edges", [])
            edge_keys = set()
            for edge in existing_edges:
                key = (edge["node_a_id"], edge["node_b_id"], edge["description"])
                edge_keys.add(key)
            new_edges = []
            for edge in new_data.get("edges", []):
                key = (edge["node_a_id"], edge["node_b_id"], edge["description"])
                if key not in edge_keys:
                    new_edges.append(edge)
                    edge_keys.add(key)
            merged_edges = existing_edges + new_edges
            merged_data = {"nodes": merged_nodes, "edges": merged_edges}
            with open(filename, "w") as f:
                json.dump(merged_data, f, indent=4)
            print("Graph saved to", filename)

    def load_json(self):
        filename, _ = QFileDialog.getOpenFileName(self, "Load Graph", "", "JSON Files (*.json)")
        if filename:
            with open(filename, "r") as f:
                data = json.load(f)
            new_default = GraphManager()
            new_default.load_from_dict(data)
            self.sessions = {"default_loaded": new_default}
            self.active_session_key = "default_loaded"
            self.session_selector.clear()
            self.session_selector.addItem("Combined View", None)
            self.session_selector.addItem("Session: default_loaded", "default_loaded")
            self.active_graph_manager = new_default
            self.graph_widget.refresh(self.active_graph_manager)
            self.data_view_widget.set_graph_manager(new_default)
            self.data_view_widget.refresh_data()
            print("Graph loaded from", filename)

# =============================================================================
# Dark Palette and Global Style Sheet
# =============================================================================

def set_dark_palette(app):
    dark_palette = QPalette()
    dark_palette.setColor(QPalette.Window, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.WindowText, Qt.white)
    dark_palette.setColor(QPalette.Base, QColor(25, 25, 25))
    dark_palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ToolTipBase, Qt.white)
    dark_palette.setColor(QPalette.ToolTipText, Qt.white)
    dark_palette.setColor(QPalette.Text, Qt.white)
    dark_palette.setColor(QPalette.Button, QColor(53, 53, 53))
    dark_palette.setColor(QPalette.ButtonText, Qt.white)
    dark_palette.setColor(QPalette.BrightText, Qt.red)
    dark_palette.setColor(QPalette.Link, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    dark_palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(dark_palette)

GLOBAL_STYLESHEET = """
    QLineEdit, QComboBox { background-color: #3c3c3c; color: white; padding: 4px; }
    QLabel { color: white; }
    QPushButton { background-color: #444444; color: white; padding: 4px; }
    QTabWidget::pane { border: 1px solid #444444; }
    QTabBar::tab { background: #3c3c3c; color: white; padding: 6px; }
    QTabBar::tab:selected { background: #444444; }
"""

# =============================================================================
# Main Application Entry Point
# =============================================================================

def main():
    app = QApplication(sys.argv)
    set_dark_palette(app)
    app.setStyleSheet(GLOBAL_STYLESHEET)
    window = MainWindow()
    window.resize(1000, 800)
    window.show()
    sys.exit(app.exec())

if __name__ == '__main__':
    main()
    