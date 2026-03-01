"""Desktop GUI for LiDAR RPi GPS Pipeline offline workflows."""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
import traceback
from typing import Any, Dict

from PySide6.QtCore import QThread, Signal
from PySide6.QtWidgets import (
    QApplication,
    QCheckBox,
    QComboBox,
    QDoubleSpinBox,
    QFileDialog,
    QFormLayout,
    QHBoxLayout,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPushButton,
    QPlainTextEdit,
    QProgressBar,
    QSpinBox,
    QTabWidget,
    QVBoxLayout,
    QWidget,
)

from lidar_rpi_gps_pipeline.jobs import JobCancelledError, run_offline_job


class JobThread(QThread):
    event_signal = Signal(dict)
    done_signal = Signal(dict)
    error_signal = Signal(str)
    cancelled_signal = Signal(str)

    def __init__(self, config: Dict[str, Any]):
        super().__init__()
        self.config = config
        self._cancel_requested = False

    def request_cancel(self) -> None:
        self._cancel_requested = True

    def _should_cancel(self) -> bool:
        return self._cancel_requested

    def run(self) -> None:
        try:
            summary = run_offline_job(
                self.config,
                emit_event=self.event_signal.emit,
                should_cancel=self._should_cancel,
            )
            self.done_signal.emit(summary)
        except JobCancelledError as e:
            self.cancelled_signal.emit(str(e))
        except Exception:
            self.error_signal.emit(traceback.format_exc())


class _BaseModeWidget(QWidget):
    def __init__(self, title: str):
        super().__init__()
        self._job_thread: JobThread | None = None
        self._last_output_osf: str | None = None
        self._viz_processes: list[subprocess.Popen] = []

        root = QVBoxLayout(self)
        root.addWidget(QLabel(title))

        self.form = QFormLayout()
        root.addLayout(self.form)

        self.processing_mode = QComboBox()
        self.processing_mode.addItem("SLAM + GPS Anchor (Recommended)", "slam_gps_anchor")
        self.processing_mode.addItem("SLAM Map (Local)", "slam_map")
        self.processing_mode.addItem("GPS Fusion", "gps_fusion")
        self.processing_mode.currentIndexChanged.connect(self._on_processing_mode_changed)
        self.form.addRow("Processing mode", self.processing_mode)

        slam_row = QHBoxLayout()
        self.slam_voxel = QDoubleSpinBox()
        self.slam_voxel.setRange(0.0, 50.0)
        self.slam_voxel.setDecimals(3)
        self.slam_voxel.setSingleStep(0.1)
        self.slam_voxel.setValue(0.0)
        self.slam_voxel.setToolTip("0.0 = auto-estimate")
        slam_row.addWidget(QLabel("Voxel (m)"))
        slam_row.addWidget(self.slam_voxel)

        self.slam_min_range = QDoubleSpinBox()
        self.slam_min_range.setRange(0.0, 100.0)
        self.slam_min_range.setDecimals(2)
        self.slam_min_range.setSingleStep(0.5)
        self.slam_min_range.setValue(1.0)
        slam_row.addWidget(QLabel("Min range (m)"))
        slam_row.addWidget(self.slam_min_range)

        self.slam_max_range = QDoubleSpinBox()
        self.slam_max_range.setRange(1.0, 500.0)
        self.slam_max_range.setDecimals(2)
        self.slam_max_range.setSingleStep(1.0)
        self.slam_max_range.setValue(150.0)
        slam_row.addWidget(QLabel("Max range (m)"))
        slam_row.addWidget(self.slam_max_range)

        self.slam_deskew = QComboBox()
        self.slam_deskew.addItems(["auto", "none", "constant_velocity", "imu_deskew"])
        slam_row.addWidget(QLabel("Deskew"))
        slam_row.addWidget(self.slam_deskew)

        slam_row.addStretch()
        slam_container = QWidget()
        slam_container.setLayout(slam_row)
        self.form.addRow("SLAM options", slam_container)

        anchor_row = QHBoxLayout()
        self.anchor_min_motion = QDoubleSpinBox()
        self.anchor_min_motion.setRange(0.0, 1000.0)
        self.anchor_min_motion.setDecimals(2)
        self.anchor_min_motion.setSingleStep(1.0)
        self.anchor_min_motion.setValue(5.0)
        anchor_row.addWidget(QLabel("Backend"))
        self.anchor_backend = QComboBox()
        self.anchor_backend.addItem("pose_optimizer (recommended)", "pose_optimizer")
        self.anchor_backend.addItem("rigid_fit", "rigid_fit")
        anchor_row.addWidget(self.anchor_backend)

        anchor_row.addWidget(QLabel("Min motion (m)"))
        anchor_row.addWidget(self.anchor_min_motion)

        self.anchor_z_mode = QComboBox()
        self.anchor_z_mode.addItems(["offset", "none"])
        anchor_row.addWidget(QLabel("Z anchor"))
        anchor_row.addWidget(self.anchor_z_mode)

        self.poseopt_constraints_every_m = QDoubleSpinBox()
        self.poseopt_constraints_every_m.setRange(0.1, 10000.0)
        self.poseopt_constraints_every_m.setDecimals(2)
        self.poseopt_constraints_every_m.setSingleStep(1.0)
        self.poseopt_constraints_every_m.setValue(10.0)
        anchor_row.addWidget(QLabel("Constraints every (m)"))
        anchor_row.addWidget(self.poseopt_constraints_every_m)

        self.poseopt_map_voxel_size = QDoubleSpinBox()
        self.poseopt_map_voxel_size.setRange(0.0, 20.0)
        self.poseopt_map_voxel_size.setDecimals(3)
        self.poseopt_map_voxel_size.setSingleStep(0.1)
        self.poseopt_map_voxel_size.setValue(0.5)
        anchor_row.addWidget(QLabel("Map voxel (m)"))
        anchor_row.addWidget(self.poseopt_map_voxel_size)
        anchor_row.addStretch()
        anchor_container = QWidget()
        anchor_container.setLayout(anchor_row)
        self.form.addRow("Anchor options", anchor_container)

        advanced_row = QHBoxLayout()
        self.time_mode = QComboBox()
        self.time_mode.addItems(["auto", "unix_ns", "relative_start"])
        advanced_row.addWidget(QLabel("Time mode"))
        advanced_row.addWidget(self.time_mode)

        self.gps_time_col = QComboBox()
        self.gps_time_col.addItems(["auto", "gps_epoch_ns", "pi_time_ns"])
        advanced_row.addWidget(QLabel("GPS time column"))
        advanced_row.addWidget(self.gps_time_col)

        self.utm_epsg = QSpinBox()
        self.utm_epsg.setRange(1000, 999999)
        self.utm_epsg.setValue(32614)
        self.utm_epsg.setToolTip("CRS EPSG written to LAS metadata and used by GPS-fusion outputs.")
        advanced_row.addWidget(QLabel("UTM EPSG"))
        advanced_row.addWidget(self.utm_epsg)

        self.save_osf = QCheckBox("Save OSF playback file")
        advanced_row.addWidget(self.save_osf)

        self.keep_converted = QCheckBox("Keep debug converted CSV files")
        advanced_row.addWidget(self.keep_converted)
        advanced_row.addStretch()
        root.addLayout(advanced_row)

        run_row = QHBoxLayout()
        self.run_btn = QPushButton("Run")
        self.run_btn.clicked.connect(self._on_run_clicked)
        run_row.addWidget(self.run_btn)

        self.stop_btn = QPushButton("Stop")
        self.stop_btn.setEnabled(False)
        self.stop_btn.clicked.connect(self._on_stop_clicked)
        run_row.addWidget(self.stop_btn)

        self.open_osf_btn = QPushButton("Open OSF in Viz")
        self.open_osf_btn.setEnabled(False)
        self.open_osf_btn.clicked.connect(self._on_open_osf_clicked)
        run_row.addWidget(self.open_osf_btn)
        run_row.addStretch()
        root.addLayout(run_row)

        self.status_label = QLabel("Idle")
        root.addWidget(self.status_label)

        self.overall_progress = QProgressBar()
        self.overall_progress.setRange(0, 1)
        self.overall_progress.setValue(0)
        self.overall_progress.setFormat("Overall: %v/%m files")
        root.addWidget(self.overall_progress)

        self.activity_progress = QProgressBar()
        self.activity_progress.setRange(0, 1)
        self.activity_progress.setValue(0)
        self.activity_progress.setFormat("Processing")
        root.addWidget(self.activity_progress)

        self.logs = QPlainTextEdit()
        self.logs.setReadOnly(True)
        root.addWidget(self.logs)
        self._on_processing_mode_changed()

    def add_path_row(self, label: str, line_edit: QLineEdit, pick_dir: bool = False, pick_file: bool = False) -> None:
        row = QHBoxLayout()
        row.addWidget(line_edit)
        pick_btn = QPushButton("Browse")
        pick_btn.clicked.connect(lambda: self._browse_path(line_edit, pick_dir=pick_dir, pick_file=pick_file))
        row.addWidget(pick_btn)
        container = QWidget()
        container.setLayout(row)
        self.form.addRow(label, container)

    def _browse_path(self, line_edit: QLineEdit, pick_dir: bool, pick_file: bool) -> None:
        if pick_dir:
            value = QFileDialog.getExistingDirectory(self, "Select directory")
        elif pick_file:
            value, _ = QFileDialog.getOpenFileName(self, "Select file")
        else:
            value = ""
        if value:
            line_edit.setText(value)

    def append_log(self, message: str) -> None:
        self.logs.appendPlainText(message)

    def current_processing_mode(self) -> str:
        return str(self.processing_mode.currentData())

    def _on_processing_mode_changed(self, _index: int = -1) -> None:
        mode = self.current_processing_mode()
        gps_needed = mode in {"gps_fusion", "slam_gps_anchor"}
        self.time_mode.setEnabled(gps_needed)
        self.gps_time_col.setEnabled(gps_needed)
        self.keep_converted.setEnabled(mode == "gps_fusion")
        self.save_osf.setEnabled(mode in {"slam_map", "slam_gps_anchor"})

        slam_mode = mode in {"slam_map", "slam_gps_anchor"}
        self.slam_voxel.setEnabled(slam_mode)
        self.slam_min_range.setEnabled(slam_mode)
        self.slam_max_range.setEnabled(slam_mode)
        self.slam_deskew.setEnabled(slam_mode)
        anchor_mode = mode == "slam_gps_anchor"
        self.anchor_min_motion.setEnabled(anchor_mode)
        self.anchor_z_mode.setEnabled(anchor_mode)
        self.anchor_backend.setEnabled(anchor_mode)
        self.poseopt_constraints_every_m.setEnabled(anchor_mode)
        self.poseopt_map_voxel_size.setEnabled(anchor_mode)

    def _set_running_ui(self, running: bool) -> None:
        self.run_btn.setEnabled(not running)
        self.stop_btn.setEnabled(running)
        self.open_osf_btn.setEnabled((not running) and bool(self._last_output_osf))
        if running:
            self.activity_progress.setRange(0, 0)
        else:
            self.activity_progress.setRange(0, 1)
            self.activity_progress.setValue(0)

    def _reset_progress(self) -> None:
        self.overall_progress.setRange(0, 1)
        self.overall_progress.setValue(0)
        self.status_label.setText("Idle")

    def _on_run_clicked(self) -> None:
        if self._job_thread is not None and self._job_thread.isRunning():
            return

        try:
            config = self.build_config()
        except ValueError as e:
            QMessageBox.warning(self, "Invalid inputs", str(e))
            return

        self._last_output_osf = None
        self._reset_progress()
        self._set_running_ui(True)
        self.append_log("Starting job...")
        self.status_label.setText("Starting")

        self._job_thread = JobThread(config)
        self._job_thread.event_signal.connect(self._on_job_event)
        self._job_thread.done_signal.connect(self._on_job_done)
        self._job_thread.error_signal.connect(self._on_job_error)
        self._job_thread.cancelled_signal.connect(self._on_job_cancelled)
        self._job_thread.start()

    def _on_stop_clicked(self) -> None:
        if self._job_thread is None or not self._job_thread.isRunning():
            return

        choice = QMessageBox.question(
            self,
            "Cancel Job",
            "Stop this run now? Partial output will be cleaned up.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if choice != QMessageBox.Yes:
            return

        self.append_log("Cancellation requested...")
        self.status_label.setText("Cancelling")
        self.stop_btn.setEnabled(False)
        self._job_thread.request_cancel()

    def _resolve_ouster_cli(self) -> str | None:
        cli = shutil.which("ouster-cli")
        if cli:
            return cli
        candidate = os.path.join(os.path.dirname(sys.executable), "ouster-cli")
        if os.path.isfile(candidate) and os.access(candidate, os.X_OK):
            return candidate
        return None

    def _on_open_osf_clicked(self) -> None:
        if not self._last_output_osf:
            QMessageBox.information(self, "No OSF", "No OSF output is available for this run.")
            return
        if not os.path.isfile(self._last_output_osf):
            QMessageBox.warning(self, "OSF Missing", f"OSF file not found:\n{self._last_output_osf}")
            return

        ouster_cli = self._resolve_ouster_cli()
        if not ouster_cli:
            QMessageBox.warning(
                self,
                "ouster-cli not found",
                "Could not find `ouster-cli` in PATH or current Python environment.",
            )
            return

        cmd = [ouster_cli, "source", self._last_output_osf, "viz"]
        try:
            proc = subprocess.Popen(cmd)
        except Exception as e:
            QMessageBox.warning(self, "Launch Failed", f"Could not start Ouster Viz:\n{e}")
            return

        self._viz_processes = [p for p in self._viz_processes if p.poll() is None]
        self._viz_processes.append(proc)
        self.append_log(f"Launched Ouster Viz for: {self._last_output_osf}")

    def _on_job_event(self, event: dict) -> None:
        kind = event.get("kind", "event")

        if kind == "status":
            message = str(event.get("message", ""))
            if message:
                self.status_label.setText(message)
                self.append_log(message)
            return

        if kind == "file_started":
            current = int(event.get("file_index", 1))
            total = max(int(event.get("file_total", 1)), 1)
            self.overall_progress.setRange(0, total)
            self.overall_progress.setValue(max(current - 1, 0))
            path = str(event.get("path", ""))
            self.status_label.setText(f"Processing file {current}/{total}")
            if path:
                self.append_log(f"Processing: {path}")
            return

        if kind == "scan_progress":
            total_points = int(event.get("total_points", 0))
            total_scans = int(event.get("total_scans", 0))
            scan_index = int(event.get("scan_index", 0))
            if total_points > 0:
                self.status_label.setText(f"Scan {scan_index} | points written: {total_points:,}")
            else:
                self.status_label.setText(f"Scan {scan_index} | scans processed: {total_scans:,}")
            return

        if kind == "file_completed":
            current = int(event.get("file_index", 1))
            total = max(int(event.get("file_total", 1)), 1)
            self.overall_progress.setRange(0, total)
            self.overall_progress.setValue(current)
            total_points = int(event.get("total_points", 0))
            total_scans = int(event.get("total_scans", 0))
            if total_points > 0:
                self.append_log(f"Completed file {current}/{total} | points: {total_points:,}")
            else:
                self.append_log(f"Completed file {current}/{total} | scans: {total_scans:,}")
            return

        if kind in {"job_started", "job_completed", "job_cancelled", "job_failed", "job_error"}:
            self.append_log(f"[{kind}] {event}")
            return

        self.append_log(f"[{kind}] {event}")

    def _on_job_done(self, summary: dict) -> None:
        self.append_log("Job completed.")
        if summary.get("output_las"):
            self.append_log(f"Output LAS: {summary['output_las']}")
        if summary.get("output_osf"):
            self._last_output_osf = str(summary["output_osf"])
            self.append_log(f"Output OSF: {self._last_output_osf}")
        if summary.get("qa_report_path"):
            self.append_log(f"QA report: {summary['qa_report_path']}")
            if summary.get("qa_pass") is not None:
                self.append_log(f"QA pass: {summary['qa_pass']}")
        if summary.get("total_points") is not None:
            self.append_log(f"Total points: {int(summary['total_points']):,}")
        if summary.get("total_scans") is not None:
            self.append_log(f"Total scans: {int(summary['total_scans']):,}")
        self.status_label.setText("Completed")
        self._set_running_ui(False)

    def _on_job_error(self, error_text: str) -> None:
        self.append_log("Job failed.")
        self.append_log(error_text)
        self.status_label.setText("Failed")
        self._set_running_ui(False)

    def _on_job_cancelled(self, reason: str) -> None:
        self.append_log(f"Job cancelled: {reason}")
        self.status_label.setText("Cancelled")
        self._set_running_ui(False)

    def build_config(self) -> Dict[str, Any]:
        raise NotImplementedError


class ManifestModeWidget(_BaseModeWidget):
    def __init__(self):
        super().__init__("Mode: Manifest -> Fuse")

        self.manifest_json = QLineEdit()
        self.manifest_base_dir = QLineEdit()
        self.gps_csv = QLineEdit()
        self.output_dir = QLineEdit()
        self.converted_csv_dir = QLineEdit()

        self.add_path_row("Manifest JSON", self.manifest_json, pick_file=True)
        self.add_path_row("Manifest base dir (optional)", self.manifest_base_dir, pick_dir=True)
        self.add_path_row("GPS CSV override (gps_fusion/anchor)", self.gps_csv, pick_file=True)
        self.add_path_row("Output dir", self.output_dir, pick_dir=True)
        self.add_path_row("Debug converted CSV dir (optional)", self.converted_csv_dir, pick_dir=True)

    def build_config(self) -> Dict[str, Any]:
        manifest = self.manifest_json.text().strip()
        out_dir = self.output_dir.text().strip()
        if not manifest:
            raise ValueError("Manifest JSON is required.")
        if not out_dir:
            raise ValueError("Output directory is required.")

        mode = self.current_processing_mode()
        cfg: Dict[str, Any] = {
            "manifest_json": manifest,
            "output_dir": out_dir,
            "processing_mode": mode,
            "utm_epsg": int(self.utm_epsg.value()),
            "slam_voxel_size": float(self.slam_voxel.value()),
            "slam_min_range": float(self.slam_min_range.value()),
            "slam_max_range": float(self.slam_max_range.value()),
            "slam_deskew_method": self.slam_deskew.currentText(),
            "anchor_min_motion_m": float(self.anchor_min_motion.value()),
            "anchor_z_mode": self.anchor_z_mode.currentText(),
            "anchor_backend": str(self.anchor_backend.currentData()),
            "poseopt_constraints_every_m": float(self.poseopt_constraints_every_m.value()),
            "poseopt_map_voxel_size": float(self.poseopt_map_voxel_size.value()),
        }
        if mode in {"slam_map", "slam_gps_anchor"}:
            cfg["save_osf"] = self.save_osf.isChecked()

        if mode in {"gps_fusion", "slam_gps_anchor"}:
            cfg["time_mode"] = self.time_mode.currentText()
            cfg["gps_time_column"] = self.gps_time_col.currentText()
        if mode == "gps_fusion":
            cfg["keep_converted"] = self.keep_converted.isChecked()

        if self.manifest_base_dir.text().strip():
            cfg["manifest_base_dir"] = self.manifest_base_dir.text().strip()
        if self.gps_csv.text().strip():
            cfg["gps_csv"] = self.gps_csv.text().strip()
        if mode == "gps_fusion" and self.converted_csv_dir.text().strip():
            cfg["converted_csv_dir"] = self.converted_csv_dir.text().strip()
        return cfg


class RawModeWidget(_BaseModeWidget):
    def __init__(self):
        super().__init__("Mode: Raw Folder -> Process (SLAM/SLAM+GPS/GPS Fusion)")

        self.raw_dir = QLineEdit()
        self.gps_csv = QLineEdit()
        self.output_dir = QLineEdit()
        self.converted_csv_dir = QLineEdit()

        self.add_path_row("Raw input dir", self.raw_dir, pick_dir=True)
        self.add_path_row("GPS CSV (required for gps_fusion/anchor)", self.gps_csv, pick_file=True)
        self.add_path_row("Output dir", self.output_dir, pick_dir=True)
        self.add_path_row("Debug converted CSV dir (optional)", self.converted_csv_dir, pick_dir=True)

    def build_config(self) -> Dict[str, Any]:
        raw_dir = self.raw_dir.text().strip()
        gps_csv = self.gps_csv.text().strip()
        out_dir = self.output_dir.text().strip()
        mode = self.current_processing_mode()

        if not raw_dir:
            raise ValueError("Raw input directory is required.")
        if mode in {"gps_fusion", "slam_gps_anchor"} and not gps_csv:
            raise ValueError("GPS CSV is required in GPS Fusion and SLAM+GPS Anchor modes.")
        if not out_dir:
            raise ValueError("Output directory is required.")

        cfg: Dict[str, Any] = {
            "lidar_raw": [raw_dir],
            "output_dir": out_dir,
            "processing_mode": mode,
            "utm_epsg": int(self.utm_epsg.value()),
            "slam_voxel_size": float(self.slam_voxel.value()),
            "slam_min_range": float(self.slam_min_range.value()),
            "slam_max_range": float(self.slam_max_range.value()),
            "slam_deskew_method": self.slam_deskew.currentText(),
            "anchor_min_motion_m": float(self.anchor_min_motion.value()),
            "anchor_z_mode": self.anchor_z_mode.currentText(),
            "anchor_backend": str(self.anchor_backend.currentData()),
            "poseopt_constraints_every_m": float(self.poseopt_constraints_every_m.value()),
            "poseopt_map_voxel_size": float(self.poseopt_map_voxel_size.value()),
        }
        if mode in {"slam_map", "slam_gps_anchor"}:
            cfg["save_osf"] = self.save_osf.isChecked()

        if gps_csv:
            cfg["gps_csv"] = gps_csv
        if mode in {"gps_fusion", "slam_gps_anchor"}:
            cfg["time_mode"] = self.time_mode.currentText()
            cfg["gps_time_column"] = self.gps_time_col.currentText()
        if mode == "gps_fusion":
            cfg["keep_converted"] = self.keep_converted.isChecked()
        if mode == "gps_fusion" and self.converted_csv_dir.text().strip():
            cfg["converted_csv_dir"] = self.converted_csv_dir.text().strip()
        return cfg


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LiDAR RPi GPS Pipeline")
        self.resize(1024, 760)

        tabs = QTabWidget()
        tabs.addTab(ManifestModeWidget(), "Manifest -> Process")
        tabs.addTab(RawModeWidget(), "Raw -> Process")
        self.setCentralWidget(tabs)


def run_gui() -> None:
    app = QApplication([])
    win = MainWindow()
    win.show()
    app.exec()
