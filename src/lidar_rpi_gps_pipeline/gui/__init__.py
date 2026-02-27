"""GUI package entrypoints."""


def run_gui() -> None:
    try:
        from .app import run_gui as _run_gui
    except ModuleNotFoundError as e:
        if getattr(e, "name", "") == "PySide6":
            raise RuntimeError(
                "PySide6 is required for GUI mode. Install it with: python -m pip install pyside6"
            ) from e
        raise

    _run_gui()


__all__ = ["run_gui"]
