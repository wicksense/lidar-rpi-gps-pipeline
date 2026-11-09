import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import process_drives  # noqa: E402

A, B, C = "20261028_101153", "20261028_101705", "20261028_102349"


def _make_steps(out, sessions):
    for name in process_drives.COMBINED_STEPS:
        os.makedirs(out / name)
    for s in sessions:
        (out / "final" / f"{process_drives.stem(s)}.las").write_bytes(b"")


def test_a_drive_added_redoes_the_steps_over_all_drives(tmp_path) -> None:
    _make_steps(tmp_path, [A, B])
    process_drives.start_drive_set(str(tmp_path), [A, B])
    assert (tmp_path / "final").exists()  # same drives: kept

    process_drives.start_drive_set(str(tmp_path), [A, B, C])

    assert not any((tmp_path / n).exists() for n in process_drives.COMBINED_STEPS)
    assert (tmp_path / "drives.txt").read_text().split() == [A, B, C]


def test_a_folder_made_before_the_record_is_judged_by_its_final_clouds(tmp_path) -> None:
    _make_steps(tmp_path, [A, B])

    process_drives.start_drive_set(str(tmp_path), [B, A])

    assert (tmp_path / "final").exists()
    assert process_drives.drives_of_previous_run(str(tmp_path)) == [A, B]


def test_a_drive_that_cannot_be_placed_is_reported_not_raised(tmp_path, monkeypatch) -> None:
    def fail(args, log_path, threads):
        with open(log_path, "w") as fh:
            fh.write("Traceback ...\nValueError: No points of 3 file(s) lie within 60 m of the route.\n")
        raise RuntimeError("failed")

    monkeypatch.setattr(process_drives, "run", fail)

    reason = process_drives.place(A, str(tmp_path), ["tiles"], 2026.82, 1)

    assert reason == "ValueError: No points of 3 file(s) lie within 60 m of the route."
