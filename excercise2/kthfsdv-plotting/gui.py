# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'guiMnyDtQ.ui'
##
## Created by: Qt User Interface Compiler version 5.15.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName("MainWindow")
        MainWindow.resize(796, 709)
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName("centralwidget")
        self.horizontalLayoutWidget = QWidget(self.centralwidget)
        self.horizontalLayoutWidget.setObjectName("horizontalLayoutWidget")
        self.horizontalLayoutWidget.setGeometry(QRect(10, 10, 759, 641))
        self.horizontalLayout = QHBoxLayout(self.horizontalLayoutWidget)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.horizontalLayout.setContentsMargins(0, 0, 0, 0)
        self.gridLayout = QGridLayout()
        self.gridLayout.setObjectName("gridLayout")
        self.label_4 = QLabel(self.horizontalLayoutWidget)
        self.label_4.setObjectName("label_4")

        self.gridLayout.addWidget(self.label_4, 8, 0, 1, 1)

        self.reset_scaling_button = QPushButton(self.horizontalLayoutWidget)
        self.reset_scaling_button.setObjectName("reset_scaling_button")

        self.gridLayout.addWidget(self.reset_scaling_button, 9, 0, 1, 1)

        self.lineEdit = QLineEdit(self.horizontalLayoutWidget)
        self.lineEdit.setObjectName("lineEdit")
        self.lineEdit.setEnabled(False)

        self.gridLayout.addWidget(self.lineEdit, 21, 1, 1, 1)

        self.line_5 = QFrame(self.horizontalLayoutWidget)
        self.line_5.setObjectName("line_5")
        self.line_5.setFrameShape(QFrame.HLine)
        self.line_5.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_5, 20, 0, 1, 3)

        self.y_max = QSpinBox(self.horizontalLayoutWidget)
        self.y_max.setObjectName("y_max")

        self.gridLayout.addWidget(self.y_max, 8, 2, 1, 1)

        self.experiment_name = QLineEdit(self.horizontalLayoutWidget)
        self.experiment_name.setObjectName("experiment_name")

        self.gridLayout.addWidget(self.experiment_name, 15, 1, 1, 1)

        self.reset_button = QPushButton(self.horizontalLayoutWidget)
        self.reset_button.setObjectName("reset_button")

        self.gridLayout.addWidget(self.reset_button, 0, 2, 1, 1)

        self.x_max = QSpinBox(self.horizontalLayoutWidget)
        self.x_max.setObjectName("x_max")

        self.gridLayout.addWidget(self.x_max, 7, 2, 1, 1)

        self.y_min = QSpinBox(self.horizontalLayoutWidget)
        self.y_min.setObjectName("y_min")

        self.gridLayout.addWidget(self.y_min, 8, 1, 1, 1)

        self.label_3 = QLabel(self.horizontalLayoutWidget)
        self.label_3.setObjectName("label_3")

        self.gridLayout.addWidget(self.label_3, 7, 0, 1, 1)

        self.zoom_slider = QSlider(self.horizontalLayoutWidget)
        self.zoom_slider.setObjectName("zoom_slider")
        self.zoom_slider.setOrientation(Qt.Horizontal)

        self.gridLayout.addWidget(self.zoom_slider, 6, 0, 1, 3)

        self.line = QFrame(self.horizontalLayoutWidget)
        self.line.setObjectName("line")
        self.line.setFrameShape(QFrame.HLine)
        self.line.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line, 2, 0, 1, 3)

        self.verticalSpacer_2 = QSpacerItem(
            20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding
        )

        self.gridLayout.addItem(self.verticalSpacer_2, 3, 0, 1, 3)

        self.save_to_csv = QPushButton(self.horizontalLayoutWidget)
        self.save_to_csv.setObjectName("save_to_csv")

        self.gridLayout.addWidget(self.save_to_csv, 18, 0, 1, 1)

        self.label = QLabel(self.horizontalLayoutWidget)
        self.label.setObjectName("label")

        self.gridLayout.addWidget(self.label, 15, 0, 1, 1)

        self.x_min = QSpinBox(self.horizontalLayoutWidget)
        self.x_min.setObjectName("x_min")

        self.gridLayout.addWidget(self.x_min, 7, 1, 1, 1)

        self.export_latex = QPushButton(self.horizontalLayoutWidget)
        self.export_latex.setObjectName("export_latex")

        self.gridLayout.addWidget(self.export_latex, 18, 1, 1, 1)

        self.show_grid = QCheckBox(self.horizontalLayoutWidget)
        self.show_grid.setObjectName("show_grid")

        self.gridLayout.addWidget(self.show_grid, 15, 2, 1, 1)

        self.line_4 = QFrame(self.horizontalLayoutWidget)
        self.line_4.setObjectName("line_4")
        self.line_4.setFrameShape(QFrame.HLine)
        self.line_4.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_4, 11, 0, 1, 3)

        self.start_button = QPushButton(self.horizontalLayoutWidget)
        self.start_button.setObjectName("start_button")

        self.gridLayout.addWidget(self.start_button, 0, 0, 1, 1)

        self.stop_button = QPushButton(self.horizontalLayoutWidget)
        self.stop_button.setObjectName("stop_button")

        self.gridLayout.addWidget(self.stop_button, 0, 1, 1, 1)

        self.verticalSpacer_3 = QSpacerItem(
            20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding
        )

        self.gridLayout.addItem(self.verticalSpacer_3, 12, 0, 1, 3)

        self.line_3 = QFrame(self.horizontalLayoutWidget)
        self.line_3.setObjectName("line_3")
        self.line_3.setFrameShape(QFrame.HLine)
        self.line_3.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_3, 4, 0, 1, 3)

        self.label_5 = QLabel(self.horizontalLayoutWidget)
        self.label_5.setObjectName("label_5")

        self.gridLayout.addWidget(self.label_5, 21, 0, 1, 1)

        self.label_2 = QLabel(self.horizontalLayoutWidget)
        self.label_2.setObjectName("label_2")

        self.gridLayout.addWidget(self.label_2, 5, 0, 1, 3)

        self.line_2 = QFrame(self.horizontalLayoutWidget)
        self.line_2.setObjectName("line_2")
        self.line_2.setFrameShape(QFrame.HLine)
        self.line_2.setFrameShadow(QFrame.Sunken)

        self.gridLayout.addWidget(self.line_2, 13, 0, 1, 3)

        self.label_6 = QLabel(self.horizontalLayoutWidget)
        self.label_6.setObjectName("label_6")

        self.gridLayout.addWidget(self.label_6, 22, 0, 1, 1)

        self.verticalSpacer = QSpacerItem(
            20, 40, QSizePolicy.Minimum, QSizePolicy.Expanding
        )

        self.gridLayout.addItem(self.verticalSpacer, 19, 0, 1, 3)

        self.horizontalLayout.addLayout(self.gridLayout)

        self.main_widget = QWidget(self.horizontalLayoutWidget)
        self.main_widget.setObjectName("main_widget")
        self.main_widget.setMinimumSize(QSize(400, 400))
        self.verticalLayoutWidget = QWidget(self.main_widget)
        self.verticalLayoutWidget.setObjectName("verticalLayoutWidget")
        self.verticalLayoutWidget.setGeometry(QRect(0, 0, 401, 641))
        self.verticalLayout_2 = QVBoxLayout(self.verticalLayoutWidget)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.verticalLayout_2.setContentsMargins(0, 0, 0, 0)
        self.plot_area = QWidget(self.verticalLayoutWidget)
        self.plot_area.setObjectName("plot_area")

        self.verticalLayout_2.addWidget(self.plot_area)

        self.spectrum_area = QWidget(self.verticalLayoutWidget)
        self.spectrum_area.setObjectName("spectrum_area_2")

        self.verticalLayout_2.addWidget(self.spectrum_area)

        self.horizontalLayout.addWidget(self.main_widget)

        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QMenuBar(MainWindow)
        self.menubar.setObjectName("menubar")
        self.menubar.setGeometry(QRect(0, 0, 796, 22))
        MainWindow.setMenuBar(self.menubar)
        self.statusbar = QStatusBar(MainWindow)
        self.statusbar.setObjectName("statusbar")
        MainWindow.setStatusBar(self.statusbar)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)

    # setupUi

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(
            QCoreApplication.translate("MainWindow", "MainWindow", None)
        )
        self.label_4.setText(QCoreApplication.translate("MainWindow", "Y-Axis", None))
        self.reset_scaling_button.setText(
            QCoreApplication.translate("MainWindow", "Reset scaling", None)
        )
        self.reset_button.setText(
            QCoreApplication.translate("MainWindow", "Reset", None)
        )
        self.label_3.setText(QCoreApplication.translate("MainWindow", "X-Axis", None))
        self.save_to_csv.setText(QCoreApplication.translate("MainWindow", "Save", None))
        self.label.setText(QCoreApplication.translate("MainWindow", "Exp. Name", None))
        self.export_latex.setText(
            QCoreApplication.translate("MainWindow", "Export", None)
        )
        self.show_grid.setText(
            QCoreApplication.translate("MainWindow", "Show Grid", None)
        )
        self.start_button.setText(
            QCoreApplication.translate("MainWindow", "Start", None)
        )
        self.stop_button.setText(QCoreApplication.translate("MainWindow", "Stop", None))
        self.label_5.setText(QCoreApplication.translate("MainWindow", "Period:", None))
        self.label_2.setText(QCoreApplication.translate("MainWindow", "Zoom", None))
        self.label_6.setText(
            QCoreApplication.translate("MainWindow", "Amplitude:", None)
        )

    # retranslateUi
