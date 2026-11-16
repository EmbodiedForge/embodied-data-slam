"""Headless replacement for Kalibr's wx plotting helper."""
class PlotCollection:
    def __init__(self, window_name="", window_size=(800, 600)):
        self.figureList = {}
    def add_figure(self, tabname, fig):
        self.figureList[tabname] = fig
    def delete_figure(self, name):
        self.figureList.pop(name, None)
    def show(self):
        return
