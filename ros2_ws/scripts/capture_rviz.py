"""QA only: capture a specified RViz X11 window, not the user's desktop."""
import sys
from PyQt5.QtWidgets import QApplication
app = QApplication([])
window = int(sys.argv[1], 0)
image = app.primaryScreen().grabWindow(window)
if image.isNull() or not image.save(sys.argv[2]):
    raise SystemExit('Unable to capture RViz window')
