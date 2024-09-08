import sys
import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QFileDialog
from PyQt5.QtCore import QTimer
from gui import Ui_MainWindow
import numpy as np
import csv
from datetime import datetime
import matplotlib.pyplot as plt
import matplotlib
from data_source import DataSource
from scipy.fftpack import fft


def signal_function(x):
    """Generates a sine-based signal"""
    return 3 * np.pi * np.exp(-(5 * np.sin(2 * np.pi * x)))


class MainWindow(QMainWindow):
    """PyQT5 GUI main window."""

    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Connect UI elements to their respective handlers
        self.ui.start_button.clicked.connect(self.handle_start)
        self.ui.stop_button.clicked.connect(self.handle_stop)
        self.ui.reset_button.clicked.connect(self.handle_reset)
        self.ui.reset_scaling_button.clicked.connect(self.toggle_auto_scaling)
        self.ui.save_to_csv.clicked.connect(self.save_to_csv)
        self.ui.export_latex.clicked.connect(self.export_to_latex)
        self.ui.show_grid.stateChanged.connect(self.toggle_grid)
        self.ui.zoom_slider.valueChanged.connect(self.handle_zoom)
        self.ui.x_min.valueChanged.connect(self.set_x_min)
        self.ui.x_max.valueChanged.connect(self.set_x_max)
        self.ui.y_min.valueChanged.connect(self.set_y_min)
        self.ui.y_max.valueChanged.connect(self.set_y_max)

        # Plotting area setup
        self.plot_widget = pg.PlotWidget()
        plot_layout = QVBoxLayout(self.ui.plot_area)
        plot_layout.addWidget(self.plot_widget)
        self.plot_widget.setTitle("h(t)", color="w", size="15pt")
        self.plot_widget.setLabel("left", "h(t)")
        self.plot_widget.setLabel("bottom", "t (s)")
        self.plot_curve = self.plot_widget.plot(pen="r")

        # Spectrum area setup
        self.spectrum_widget = pg.PlotWidget()
        spectrum_layout = QVBoxLayout(self.ui.spectrum_area)
        spectrum_layout.addWidget(self.spectrum_widget)
        self.spectrum_widget.setTitle("Spectrum", color="w", size="15pt")
        self.spectrum_widget.setLabel("left", "Amplitude")
        self.spectrum_widget.setLabel("bottom", "Frequency (Hz)")
        self.spectrum_widget.addItem(self.generate_spectrum_bars())

        # Data source and timer setup
        self.update_interval_ms = 20  # Update interval in milliseconds
        self.data_source = DataSource(signal_function, self.update_interval_ms)
        self.timer = QTimer()
        self.timer.setInterval(self.update_interval_ms)
        self.timer.timeout.connect(self.update_plot)
        self.timer.start()

    def update_plot(self):
        """UAdds new Data to the DataSource class and updates the plot."""
        self.data_source.measure()
        self.plot_curve.setData(
            self.data_source.get_time_values(), self.data_source.get_signal_values()
        )

    def generate_spectrum_bars(self):
        """Generates FFT spectrum as bar graph"""
        sample_rate = 500
        time_values = np.linspace(0, 1, sample_rate)
        fft_values = fft(signal_function(time_values))
        bar_item = pg.BarGraphItem(
            x=time_values, height=np.abs(fft_values), width=0.01, brush="b"
        )
        return bar_item

    def handle_start(self):
        """Starts the timer to update the plot"""
        self.timer.start()

    def handle_stop(self):
        """Stops the timer to pause the plot updates"""
        self.timer.stop()

    def handle_reset(self):
        """Resets the data and updates the plot"""
        self.data_source.reset()
        self.update_plot()

    def toggle_grid(self, state):
        """Toggles the grid visibility"""
        show_grid = state == 2  # qt5 checkbox: 2 if checked, 0 otherwise
        self.plot_widget.showGrid(x=show_grid, y=show_grid)

    def handle_zoom(self, value):
        """Adjusts X-axis zoom based on the QSlider value"""
        adjusted_value = (
            100 - value
        )  # qt5 slider: range from 0 to 99, adjusted value: from 1 to 100
        self.plot_widget.enableAutoRange(
            axis=pg.ViewBox.XAxis, enable=adjusted_value / 100
        )

    def set_x_min(self, value):
        """Sets the minimum value of the X-axis"""
        self.plot_widget.setLimits(xMin=value)

    def set_x_max(self, value):
        """Sets the maximum value of the X-axis"""
        self.plot_widget.setLimits(xMax=value)

    def set_y_min(self, value):
        """Sets the minimum value of the Y-axis"""
        self.plot_widget.setLimits(yMin=value)

    def set_y_max(self, value):
        """Sets the maximum value of the Y-axis"""
        self.plot_widget.setLimits(yMax=value)

    def toggle_auto_scaling(self):
        """Resets axis limits to (-1e+307,1e+307)"""
        self.ui.x_min.setValue(0)
        self.ui.x_max.setValue(0)
        self.ui.y_min.setValue(0)
        self.ui.y_max.setValue(0)
        self.plot_widget.setLimits(xMin=-1e307, xMax=1e307, yMin=-1e307, yMax=1e307)

    def save_to_csv(self):
        """Saves the data to a CSV file with ISO-format timestamp and experiment name as header and columnnames afterwards."""
        file_name = self.get_save_file_name()
        if file_name:
            with open(file_name, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(
                    [f"{datetime.now()}", f"{self.ui.experiment_name.text()}"]
                )
                writer.writerow(["t", "h(t)"])
                time_data, signal_data = self.data_source[:]
                for time_val, signal_val in zip(time_data, signal_data):
                    writer.writerow([time_val, signal_val])

    def export_to_latex(self):
        """Exports the plot as a LaTeX-compatible figure"""
        matplotlib.use("pgf")
        matplotlib.rcParams.update(
            {
                "pgf.texsystem": "pdflatex",
                "font.family": "serif",
                "text.usetex": True,
                "pgf.rcfonts": False,
            }
        )
        time_data, signal_data = self.data_source[:]
        plt.plot(time_data, signal_data, "-")
        plt.xlabel("t")
        plt.ylabel("h(t)")
        plt.title(f"Function Plot - {self.ui.experiment_name.text()}")
        plt.grid(True)
        plt.show()
        file_name = self.get_save_file_name()
        plt.savefig(file_name)

    def get_save_file_name(self):
        """Opens a file save dialog and returns the selected file name"""
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        file_name, _ = QFileDialog.getSaveFileName(
            self, "Save File", "", "All Files (*);;Text Files (*.txt)", options=options
        )
        return file_name


if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()
    sys.exit(app.exec_())
