from pathlib import Path


def test_main_does_not_override_fixed_map_tolerances_with_defaults():
    repo_root = Path(__file__).resolve().parents[2]
    main_c = repo_root / "user" / "src" / "main.c"
    source = main_c.read_text(encoding="utf-8")

    init_pos = source.index("motion_exec_init();")
    pulse_mode_pos = source.index("#if SMARTCAR_RUNTIME_MODE == SMARTCAR_MODE_PULSE_CAL_3200", init_pos)
    post_init_common = source[init_pos:pulse_mode_pos]

    assert "motion_exec_set_segment_accept_tolerance_mm(EXEC_SEGMENT_ACCEPT_TOL_MM);" not in post_init_common
    assert "odometry_set_finish_tolerance_mm(POINT_MOVE_FINISH_TOL_MM);" not in post_init_common
