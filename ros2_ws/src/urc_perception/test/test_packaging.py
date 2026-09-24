import ast
from pathlib import Path
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]


def test_one_flat_ament_python_package():
    manifests = list(ROOT.rglob('package.xml'))
    assert manifests == [ROOT / 'package.xml']
    package = ET.parse(manifests[0]).getroot()
    assert package.findtext('name') == 'urc_perception'
    assert package.findtext('export/build_type') == 'ament_python'
    assert 'urc_interfaces' in [item.text for item in package.findall('exec_depend')]
    assert (ROOT / 'resource/urc_perception').is_file()


def test_all_python_sources_parse_without_ros():
    for source in ROOT.rglob('*.py'):
        ast.parse(source.read_text(encoding='utf-8'), filename=str(source))
