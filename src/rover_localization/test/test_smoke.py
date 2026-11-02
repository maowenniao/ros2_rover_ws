def test_package_marker_exists():
    from pathlib import Path
    assert (Path(__file__).resolve().parents[1] / 'package.xml').is_file()
