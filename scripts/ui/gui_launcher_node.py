# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'gui_launcher_node.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        MainWindow.setObjectName(_fromUtf8("MainWindow"))
        MainWindow.resize(762, 409)
        self.centralwidget = QtGui.QWidget(MainWindow)
        self.centralwidget.setObjectName(_fromUtf8("centralwidget"))
        self.horizontalLayout_3 = QtGui.QHBoxLayout(self.centralwidget)
        self.horizontalLayout_3.setObjectName(_fromUtf8("horizontalLayout_3"))
        self.tabWidget = QtGui.QTabWidget(self.centralwidget)
        self.tabWidget.setObjectName(_fromUtf8("tabWidget"))
        self.tabLaunch = QtGui.QWidget()
        font = QtGui.QFont()
        font.setPointSize(11)
        self.tabLaunch.setFont(font)
        self.tabLaunch.setObjectName(_fromUtf8("tabLaunch"))
        self.gridLayoutLaunch = QtGui.QGridLayout(self.tabLaunch)
        self.gridLayoutLaunch.setMargin(10)
        self.gridLayoutLaunch.setObjectName(_fromUtf8("gridLayoutLaunch"))
        self.hLayoutLaunch = QtGui.QHBoxLayout()
        self.hLayoutLaunch.setObjectName(_fromUtf8("hLayoutLaunch"))
        self.vLayoutLaunchList = QtGui.QVBoxLayout()
        self.vLayoutLaunchList.setObjectName(_fromUtf8("vLayoutLaunchList"))
        self.hLayoutListHeader = QtGui.QHBoxLayout()
        self.hLayoutListHeader.setObjectName(_fromUtf8("hLayoutListHeader"))
        self.labelLaunchListHeader = QtGui.QLabel(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelLaunchListHeader.sizePolicy().hasHeightForWidth())
        self.labelLaunchListHeader.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(14)
        font.setBold(True)
        font.setItalic(False)
        font.setWeight(75)
        self.labelLaunchListHeader.setFont(font)
        self.labelLaunchListHeader.setObjectName(_fromUtf8("labelLaunchListHeader"))
        self.hLayoutListHeader.addWidget(self.labelLaunchListHeader, 0, QtCore.Qt.AlignVCenter)
        self.labelListInfo = QtGui.QLabel(self.tabLaunch)
        self.labelListInfo.setObjectName(_fromUtf8("labelListInfo"))
        self.hLayoutListHeader.addWidget(self.labelListInfo, 0, QtCore.Qt.AlignVCenter)
        self.btnClearLaunchList = QtGui.QPushButton(self.tabLaunch)
        icon = QtGui.QIcon()
        icon.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/New document.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnClearLaunchList.setIcon(icon)
        self.btnClearLaunchList.setObjectName(_fromUtf8("btnClearLaunchList"))
        self.hLayoutListHeader.addWidget(self.btnClearLaunchList)
        self.vLayoutLaunchList.addLayout(self.hLayoutListHeader)
        self.robotTree = QtGui.QTreeWidget(self.tabLaunch)
        self.robotTree.setAlternatingRowColors(True)
        self.robotTree.setObjectName(_fromUtf8("robotTree"))
        item_0 = QtGui.QTreeWidgetItem(self.robotTree)
        item_0 = QtGui.QTreeWidgetItem(self.robotTree)
        item_0 = QtGui.QTreeWidgetItem(self.robotTree)
        self.robotTree.header().setStretchLastSection(True)
        self.vLayoutLaunchList.addWidget(self.robotTree)
        self.hLayoutAddRobot = QtGui.QHBoxLayout()
        self.hLayoutAddRobot.setObjectName(_fromUtf8("hLayoutAddRobot"))
        self.comboBoxRobotType = QtGui.QComboBox(self.tabLaunch)
        self.comboBoxRobotType.setMinimumSize(QtCore.QSize(153, 0))
        self.comboBoxRobotType.setObjectName(_fromUtf8("comboBoxRobotType"))
        self.comboBoxRobotType.addItem(_fromUtf8(""))
        self.comboBoxRobotType.addItem(_fromUtf8(""))
        self.comboBoxRobotType.addItem(_fromUtf8(""))
        self.hLayoutAddRobot.addWidget(self.comboBoxRobotType)
        self.spinBoxRobotID = QtGui.QSpinBox(self.tabLaunch)
        self.spinBoxRobotID.setPrefix(_fromUtf8(""))
        self.spinBoxRobotID.setMinimum(1)
        self.spinBoxRobotID.setObjectName(_fromUtf8("spinBoxRobotID"))
        self.hLayoutAddRobot.addWidget(self.spinBoxRobotID)
        self.checkBoxSFMMPDM = QtGui.QCheckBox(self.tabLaunch)
        self.checkBoxSFMMPDM.setObjectName(_fromUtf8("checkBoxSFMMPDM"))
        self.hLayoutAddRobot.addWidget(self.checkBoxSFMMPDM)
        self.comboBoxDockingStationLaunch = QtGui.QComboBox(self.tabLaunch)
        self.comboBoxDockingStationLaunch.setMinimumSize(QtCore.QSize(153, 0))
        self.comboBoxDockingStationLaunch.setObjectName(_fromUtf8("comboBoxDockingStationLaunch"))
        self.comboBoxDockingStationLaunch.addItem(_fromUtf8(""))
        self.comboBoxDockingStationLaunch.addItem(_fromUtf8(""))
        self.comboBoxDockingStationLaunch.addItem(_fromUtf8(""))
        self.hLayoutAddRobot.addWidget(self.comboBoxDockingStationLaunch)
        self.btnAddRobot = QtGui.QPushButton(self.tabLaunch)
        icon1 = QtGui.QIcon()
        icon1.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Add.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnAddRobot.setIcon(icon1)
        self.btnAddRobot.setObjectName(_fromUtf8("btnAddRobot"))
        self.hLayoutAddRobot.addWidget(self.btnAddRobot)
        self.vLayoutLaunchList.addLayout(self.hLayoutAddRobot)
        self.hLayoutLaunch.addLayout(self.vLayoutLaunchList)
        self.vLayoutLaunchControl = QtGui.QVBoxLayout()
        self.vLayoutLaunchControl.setObjectName(_fromUtf8("vLayoutLaunchControl"))
        self.labelStatusTitle = QtGui.QLabel(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelStatusTitle.sizePolicy().hasHeightForWidth())
        self.labelStatusTitle.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setBold(True)
        font.setWeight(75)
        self.labelStatusTitle.setFont(font)
        self.labelStatusTitle.setObjectName(_fromUtf8("labelStatusTitle"))
        self.vLayoutLaunchControl.addWidget(self.labelStatusTitle)
        self.labelStatusText = QtGui.QLabel(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelStatusText.sizePolicy().hasHeightForWidth())
        self.labelStatusText.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setItalic(True)
        self.labelStatusText.setFont(font)
        self.labelStatusText.setObjectName(_fromUtf8("labelStatusText"))
        self.vLayoutLaunchControl.addWidget(self.labelStatusText, 0, QtCore.Qt.AlignHCenter)
        self.labelStatusIcon = QtGui.QLabel(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Maximum)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelStatusIcon.sizePolicy().hasHeightForWidth())
        self.labelStatusIcon.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setItalic(True)
        self.labelStatusIcon.setFont(font)
        self.labelStatusIcon.setText(_fromUtf8(""))
        self.labelStatusIcon.setPixmap(QtGui.QPixmap(_fromUtf8(":/icons/How-to.png")))
        self.labelStatusIcon.setAlignment(QtCore.Qt.AlignCenter)
        self.labelStatusIcon.setObjectName(_fromUtf8("labelStatusIcon"))
        self.vLayoutLaunchControl.addWidget(self.labelStatusIcon)
        spacerItem = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Maximum)
        self.vLayoutLaunchControl.addItem(spacerItem)
        self.btnShutdown = QtGui.QPushButton(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Preferred, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnShutdown.sizePolicy().hasHeightForWidth())
        self.btnShutdown.setSizePolicy(sizePolicy)
        icon2 = QtGui.QIcon()
        icon2.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Abort.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnShutdown.setIcon(icon2)
        self.btnShutdown.setObjectName(_fromUtf8("btnShutdown"))
        self.vLayoutLaunchControl.addWidget(self.btnShutdown)
        self.btnLaunch = QtGui.QPushButton(self.tabLaunch)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.btnLaunch.sizePolicy().hasHeightForWidth())
        self.btnLaunch.setSizePolicy(sizePolicy)
        self.btnLaunch.setLayoutDirection(QtCore.Qt.LeftToRight)
        icon3 = QtGui.QIcon()
        icon3.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Next.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.btnLaunch.setIcon(icon3)
        self.btnLaunch.setObjectName(_fromUtf8("btnLaunch"))
        self.vLayoutLaunchControl.addWidget(self.btnLaunch)
        self.hLayoutLaunch.addLayout(self.vLayoutLaunchControl)
        self.gridLayoutLaunch.addLayout(self.hLayoutLaunch, 0, 0, 1, 1)
        self.tabWidget.addTab(self.tabLaunch, _fromUtf8(""))
        self.tabEditDockingStation = QtGui.QWidget()
        self.tabEditDockingStation.setObjectName(_fromUtf8("tabEditDockingStation"))
        self.gridLayoutDockingStation = QtGui.QGridLayout(self.tabEditDockingStation)
        self.gridLayoutDockingStation.setMargin(10)
        self.gridLayoutDockingStation.setObjectName(_fromUtf8("gridLayoutDockingStation"))
        self.vLayoutDockingStation = QtGui.QVBoxLayout()
        self.vLayoutDockingStation.setObjectName(_fromUtf8("vLayoutDockingStation"))
        self.hLayoutRobotType = QtGui.QHBoxLayout()
        self.hLayoutRobotType.setObjectName(_fromUtf8("hLayoutRobotType"))
        self.labelDockingStationID = QtGui.QLabel(self.tabEditDockingStation)
        self.labelDockingStationID.setObjectName(_fromUtf8("labelDockingStationID"))
        self.hLayoutRobotType.addWidget(self.labelDockingStationID)
        self.comboBoxDockingStationID = QtGui.QComboBox(self.tabEditDockingStation)
        self.comboBoxDockingStationID.setMinimumSize(QtCore.QSize(153, 0))
        self.comboBoxDockingStationID.setObjectName(_fromUtf8("comboBoxDockingStationID"))
        self.comboBoxDockingStationID.addItem(_fromUtf8(""))
        self.comboBoxDockingStationID.addItem(_fromUtf8(""))
        self.comboBoxDockingStationID.addItem(_fromUtf8(""))
        self.hLayoutRobotType.addWidget(self.comboBoxDockingStationID)
        self.vLayoutDockingStation.addLayout(self.hLayoutRobotType)
        spacerItem1 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vLayoutDockingStation.addItem(spacerItem1)
        self.labelDockingStationTitle = QtGui.QLabel(self.tabEditDockingStation)
        self.labelDockingStationTitle.setObjectName(_fromUtf8("labelDockingStationTitle"))
        self.vLayoutDockingStation.addWidget(self.labelDockingStationTitle)
        self.hLayoutDockingStationOrigin = QtGui.QHBoxLayout()
        self.hLayoutDockingStationOrigin.setObjectName(_fromUtf8("hLayoutDockingStationOrigin"))
        self.labelDockingStationOriginX = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelDockingStationOriginX.sizePolicy().hasHeightForWidth())
        self.labelDockingStationOriginX.setSizePolicy(sizePolicy)
        self.labelDockingStationOriginX.setObjectName(_fromUtf8("labelDockingStationOriginX"))
        self.hLayoutDockingStationOrigin.addWidget(self.labelDockingStationOriginX)
        self.doubleSpinBoxDockingStationOriginX = QtGui.QDoubleSpinBox(self.tabEditDockingStation)
        self.doubleSpinBoxDockingStationOriginX.setDecimals(2)
        self.doubleSpinBoxDockingStationOriginX.setMinimum(-100.0)
        self.doubleSpinBoxDockingStationOriginX.setObjectName(_fromUtf8("doubleSpinBoxDockingStationOriginX"))
        self.hLayoutDockingStationOrigin.addWidget(self.doubleSpinBoxDockingStationOriginX)
        self.labelDockingStationOriginY = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelDockingStationOriginY.sizePolicy().hasHeightForWidth())
        self.labelDockingStationOriginY.setSizePolicy(sizePolicy)
        self.labelDockingStationOriginY.setObjectName(_fromUtf8("labelDockingStationOriginY"))
        self.hLayoutDockingStationOrigin.addWidget(self.labelDockingStationOriginY)
        self.doubleSpinBoxDockingStationOriginY = QtGui.QDoubleSpinBox(self.tabEditDockingStation)
        self.doubleSpinBoxDockingStationOriginY.setDecimals(2)
        self.doubleSpinBoxDockingStationOriginY.setMinimum(-100.0)
        self.doubleSpinBoxDockingStationOriginY.setObjectName(_fromUtf8("doubleSpinBoxDockingStationOriginY"))
        self.hLayoutDockingStationOrigin.addWidget(self.doubleSpinBoxDockingStationOriginY)
        self.labelDockingStationOriginTheta = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelDockingStationOriginTheta.sizePolicy().hasHeightForWidth())
        self.labelDockingStationOriginTheta.setSizePolicy(sizePolicy)
        self.labelDockingStationOriginTheta.setObjectName(_fromUtf8("labelDockingStationOriginTheta"))
        self.hLayoutDockingStationOrigin.addWidget(self.labelDockingStationOriginTheta)
        self.spinBoxDockingStationOriginTheta = QtGui.QSpinBox(self.tabEditDockingStation)
        self.spinBoxDockingStationOriginTheta.setMaximum(359)
        self.spinBoxDockingStationOriginTheta.setSingleStep(1)
        self.spinBoxDockingStationOriginTheta.setObjectName(_fromUtf8("spinBoxDockingStationOriginTheta"))
        self.hLayoutDockingStationOrigin.addWidget(self.spinBoxDockingStationOriginTheta)
        self.vLayoutDockingStation.addLayout(self.hLayoutDockingStationOrigin)
        spacerItem2 = QtGui.QSpacerItem(20, 20, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Fixed)
        self.vLayoutDockingStation.addItem(spacerItem2)
        self.labelDockingCellOptions = QtGui.QLabel(self.tabEditDockingStation)
        self.labelDockingCellOptions.setObjectName(_fromUtf8("labelDockingCellOptions"))
        self.vLayoutDockingStation.addWidget(self.labelDockingCellOptions)
        self.hLayoutCellHeaders = QtGui.QHBoxLayout()
        self.hLayoutCellHeaders.setObjectName(_fromUtf8("hLayoutCellHeaders"))
        self.labelCellSpacing = QtGui.QLabel(self.tabEditDockingStation)
        self.labelCellSpacing.setObjectName(_fromUtf8("labelCellSpacing"))
        self.hLayoutCellHeaders.addWidget(self.labelCellSpacing)
        self.labelCellAngle = QtGui.QLabel(self.tabEditDockingStation)
        self.labelCellAngle.setObjectName(_fromUtf8("labelCellAngle"))
        self.hLayoutCellHeaders.addWidget(self.labelCellAngle, 0, QtCore.Qt.AlignHCenter)
        self.vLayoutDockingStation.addLayout(self.hLayoutCellHeaders)
        self.hLayoutRobotOffsetSpacing = QtGui.QHBoxLayout()
        self.hLayoutRobotOffsetSpacing.setObjectName(_fromUtf8("hLayoutRobotOffsetSpacing"))
        self.labelRobotOffsetSpacingX = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelRobotOffsetSpacingX.sizePolicy().hasHeightForWidth())
        self.labelRobotOffsetSpacingX.setSizePolicy(sizePolicy)
        self.labelRobotOffsetSpacingX.setObjectName(_fromUtf8("labelRobotOffsetSpacingX"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.labelRobotOffsetSpacingX)
        self.doubleSpinBoxRobotOffsetSpacingX = QtGui.QDoubleSpinBox(self.tabEditDockingStation)
        self.doubleSpinBoxRobotOffsetSpacingX.setDecimals(2)
        self.doubleSpinBoxRobotOffsetSpacingX.setMinimum(-100.0)
        self.doubleSpinBoxRobotOffsetSpacingX.setObjectName(_fromUtf8("doubleSpinBoxRobotOffsetSpacingX"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.doubleSpinBoxRobotOffsetSpacingX)
        self.labelRobotOffsetSpacingY = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelRobotOffsetSpacingY.sizePolicy().hasHeightForWidth())
        self.labelRobotOffsetSpacingY.setSizePolicy(sizePolicy)
        self.labelRobotOffsetSpacingY.setObjectName(_fromUtf8("labelRobotOffsetSpacingY"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.labelRobotOffsetSpacingY)
        self.doubleSpinBoxRobotOffsetSpacingY = QtGui.QDoubleSpinBox(self.tabEditDockingStation)
        self.doubleSpinBoxRobotOffsetSpacingY.setDecimals(2)
        self.doubleSpinBoxRobotOffsetSpacingY.setMinimum(-100.0)
        self.doubleSpinBoxRobotOffsetSpacingY.setObjectName(_fromUtf8("doubleSpinBoxRobotOffsetSpacingY"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.doubleSpinBoxRobotOffsetSpacingY)
        self.labelRobotOffsetSpacingTheta = QtGui.QLabel(self.tabEditDockingStation)
        sizePolicy = QtGui.QSizePolicy(QtGui.QSizePolicy.Maximum, QtGui.QSizePolicy.Preferred)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.labelRobotOffsetSpacingTheta.sizePolicy().hasHeightForWidth())
        self.labelRobotOffsetSpacingTheta.setSizePolicy(sizePolicy)
        self.labelRobotOffsetSpacingTheta.setCursor(QtGui.QCursor(QtCore.Qt.ArrowCursor))
        self.labelRobotOffsetSpacingTheta.setObjectName(_fromUtf8("labelRobotOffsetSpacingTheta"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.labelRobotOffsetSpacingTheta)
        self.spinBoxRobotOffsetSpacingTheta = QtGui.QSpinBox(self.tabEditDockingStation)
        self.spinBoxRobotOffsetSpacingTheta.setMaximum(359)
        self.spinBoxRobotOffsetSpacingTheta.setObjectName(_fromUtf8("spinBoxRobotOffsetSpacingTheta"))
        self.hLayoutRobotOffsetSpacing.addWidget(self.spinBoxRobotOffsetSpacingTheta)
        self.vLayoutDockingStation.addLayout(self.hLayoutRobotOffsetSpacing)
        self.hLayoutRobotsPerRow = QtGui.QHBoxLayout()
        self.hLayoutRobotsPerRow.setObjectName(_fromUtf8("hLayoutRobotsPerRow"))
        self.labelRobotsPerRow = QtGui.QLabel(self.tabEditDockingStation)
        self.labelRobotsPerRow.setObjectName(_fromUtf8("labelRobotsPerRow"))
        self.hLayoutRobotsPerRow.addWidget(self.labelRobotsPerRow)
        self.spinBoxRobotsPerRow = QtGui.QSpinBox(self.tabEditDockingStation)
        self.spinBoxRobotsPerRow.setMinimum(1)
        self.spinBoxRobotsPerRow.setProperty("value", 1)
        self.spinBoxRobotsPerRow.setObjectName(_fromUtf8("spinBoxRobotsPerRow"))
        self.hLayoutRobotsPerRow.addWidget(self.spinBoxRobotsPerRow)
        self.vLayoutDockingStation.addLayout(self.hLayoutRobotsPerRow)
        self.hLayoutRobotsPerColumn = QtGui.QHBoxLayout()
        self.hLayoutRobotsPerColumn.setObjectName(_fromUtf8("hLayoutRobotsPerColumn"))
        self.labelRobotsPerColumn = QtGui.QLabel(self.tabEditDockingStation)
        self.labelRobotsPerColumn.setObjectName(_fromUtf8("labelRobotsPerColumn"))
        self.hLayoutRobotsPerColumn.addWidget(self.labelRobotsPerColumn)
        self.spinBoxRobotsPerColumn = QtGui.QSpinBox(self.tabEditDockingStation)
        self.spinBoxRobotsPerColumn.setMinimum(1)
        self.spinBoxRobotsPerColumn.setProperty("value", 1)
        self.spinBoxRobotsPerColumn.setObjectName(_fromUtf8("spinBoxRobotsPerColumn"))
        self.hLayoutRobotsPerColumn.addWidget(self.spinBoxRobotsPerColumn)
        self.vLayoutDockingStation.addLayout(self.hLayoutRobotsPerColumn)
        spacerItem3 = QtGui.QSpacerItem(20, 40, QtGui.QSizePolicy.Minimum, QtGui.QSizePolicy.Expanding)
        self.vLayoutDockingStation.addItem(spacerItem3)
        self.gridLayoutDockingStation.addLayout(self.vLayoutDockingStation, 0, 0, 1, 1)
        self.tabWidget.addTab(self.tabEditDockingStation, _fromUtf8(""))
        self.horizontalLayout_3.addWidget(self.tabWidget)
        MainWindow.setCentralWidget(self.centralwidget)
        self.menubar = QtGui.QMenuBar(MainWindow)
        self.menubar.setGeometry(QtCore.QRect(0, 0, 762, 22))
        self.menubar.setObjectName(_fromUtf8("menubar"))
        self.menuFile = QtGui.QMenu(self.menubar)
        self.menuFile.setObjectName(_fromUtf8("menuFile"))
        MainWindow.setMenuBar(self.menubar)
        self.statusBar = QtGui.QStatusBar(MainWindow)
        self.statusBar.setObjectName(_fromUtf8("statusBar"))
        MainWindow.setStatusBar(self.statusBar)
        self.actionSave_file = QtGui.QAction(MainWindow)
        icon4 = QtGui.QIcon()
        icon4.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Save.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionSave_file.setIcon(icon4)
        self.actionSave_file.setObjectName(_fromUtf8("actionSave_file"))
        self.actionLoad_file = QtGui.QAction(MainWindow)
        icon5 = QtGui.QIcon()
        icon5.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Folder.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionLoad_file.setIcon(icon5)
        self.actionLoad_file.setObjectName(_fromUtf8("actionLoad_file"))
        self.actionQuit_application = QtGui.QAction(MainWindow)
        icon6 = QtGui.QIcon()
        icon6.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/Close.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionQuit_application.setIcon(icon6)
        self.actionQuit_application.setObjectName(_fromUtf8("actionQuit_application"))
        self.actionNew = QtGui.QAction(MainWindow)
        self.actionNew.setIcon(icon)
        self.actionNew.setObjectName(_fromUtf8("actionNew"))
        self.actionAbout = QtGui.QAction(MainWindow)
        icon7 = QtGui.QIcon()
        icon7.addPixmap(QtGui.QPixmap(_fromUtf8(":/icons/About.png")), QtGui.QIcon.Normal, QtGui.QIcon.Off)
        self.actionAbout.setIcon(icon7)
        self.actionAbout.setObjectName(_fromUtf8("actionAbout"))
        self.menuFile.addAction(self.actionNew)
        self.menuFile.addAction(self.actionSave_file)
        self.menuFile.addAction(self.actionLoad_file)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionAbout)
        self.menuFile.addSeparator()
        self.menuFile.addAction(self.actionQuit_application)
        self.menubar.addAction(self.menuFile.menuAction())

        self.retranslateUi(MainWindow)
        self.tabWidget.setCurrentIndex(0)
        QtCore.QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(_translate("MainWindow", "multi_robot_sim - Launcher GUI", None))
        self.labelLaunchListHeader.setText(_translate("MainWindow", "Launch list", None))
        self.labelListInfo.setText(_translate("MainWindow", "(RMB - remove an item)", None))
        self.btnClearLaunchList.setText(_translate("MainWindow", "  &Clear Launch list", None))
        self.robotTree.headerItem().setText(0, _translate("MainWindow", "Robot ID", None))
        self.robotTree.headerItem().setText(1, _translate("MainWindow", "Navigation", None))
        self.robotTree.headerItem().setText(2, _translate("MainWindow", "Robot type", None))
        self.robotTree.headerItem().setText(3, _translate("MainWindow", "Docking station (ds)", None))
        __sortingEnabled = self.robotTree.isSortingEnabled()
        self.robotTree.setSortingEnabled(False)
        self.robotTree.topLevelItem(0).setText(0, _translate("MainWindow", "rdg01", None))
        self.robotTree.topLevelItem(0).setText(1, _translate("MainWindow", "SFM-MPDM", None))
        self.robotTree.topLevelItem(0).setText(2, _translate("MainWindow", "Ridgeback", None))
        self.robotTree.topLevelItem(0).setText(3, _translate("MainWindow", "ds01", None))
        self.robotTree.topLevelItem(1).setText(0, _translate("MainWindow", "rdg02", None))
        self.robotTree.topLevelItem(1).setText(1, _translate("MainWindow", "SFM-MPDM", None))
        self.robotTree.topLevelItem(1).setText(2, _translate("MainWindow", "Ridgeback", None))
        self.robotTree.topLevelItem(1).setText(3, _translate("MainWindow", "ds01", None))
        self.robotTree.topLevelItem(2).setText(0, _translate("MainWindow", "rdg11", None))
        self.robotTree.topLevelItem(2).setText(1, _translate("MainWindow", "move_base", None))
        self.robotTree.topLevelItem(2).setText(2, _translate("MainWindow", "Ridgeback", None))
        self.robotTree.topLevelItem(2).setText(3, _translate("MainWindow", "ds03", None))
        self.robotTree.setSortingEnabled(__sortingEnabled)
        self.comboBoxRobotType.setItemText(0, _translate("MainWindow", "Ridgeback", None))
        self.comboBoxRobotType.setItemText(1, _translate("MainWindow", "ROSbot", None))
        self.comboBoxRobotType.setItemText(2, _translate("MainWindow", "Husky", None))
        self.checkBoxSFMMPDM.setText(_translate("MainWindow", "SFM-MPDM", None))
        self.comboBoxDockingStationLaunch.setItemText(0, _translate("MainWindow", "ds01", None))
        self.comboBoxDockingStationLaunch.setItemText(1, _translate("MainWindow", "ds02", None))
        self.comboBoxDockingStationLaunch.setItemText(2, _translate("MainWindow", "ds03", None))
        self.btnAddRobot.setText(_translate("MainWindow", "  &Add robot", None))
        self.labelStatusTitle.setText(_translate("MainWindow", "Launch status:", None))
        self.labelStatusText.setText(_translate("MainWindow", "Awaiting launch...", None))
        self.btnShutdown.setText(_translate("MainWindow", "  &Shutdown", None))
        self.btnLaunch.setText(_translate("MainWindow", "   &Launch", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabLaunch), _translate("MainWindow", "Launch", None))
        self.tabWidget.setTabToolTip(self.tabWidget.indexOf(self.tabLaunch), _translate("MainWindow", "Launch list and launch control options", None))
        self.labelDockingStationID.setText(_translate("MainWindow", "Docking station ID:", None))
        self.comboBoxDockingStationID.setItemText(0, _translate("MainWindow", "ds01", None))
        self.comboBoxDockingStationID.setItemText(1, _translate("MainWindow", "ds02", None))
        self.comboBoxDockingStationID.setItemText(2, _translate("MainWindow", "ds03", None))
        self.labelDockingStationTitle.setText(_translate("MainWindow", "Docking station origin", None))
        self.labelDockingStationOriginX.setText(_translate("MainWindow", "  X:   ", None))
        self.labelDockingStationOriginY.setText(_translate("MainWindow", "  Y:   ", None))
        self.labelDockingStationOriginTheta.setText(_translate("MainWindow", "  Theta:   ", None))
        self.labelDockingCellOptions.setText(_translate("MainWindow", "Docking cell options", None))
        self.labelCellSpacing.setText(_translate("MainWindow", "Cell spacing (distance between cell borders)", None))
        self.labelCellAngle.setText(_translate("MainWindow", "Cell angle (degrees)", None))
        self.labelRobotOffsetSpacingX.setText(_translate("MainWindow", "  X:   ", None))
        self.labelRobotOffsetSpacingY.setText(_translate("MainWindow", "  Y:   ", None))
        self.labelRobotOffsetSpacingTheta.setText(_translate("MainWindow", "  Theta:   ", None))
        self.labelRobotsPerRow.setText(_translate("MainWindow", "Maximum number of robots per row:", None))
        self.labelRobotsPerColumn.setText(_translate("MainWindow", "Maximum number of robots per column:", None))
        self.tabWidget.setTabText(self.tabWidget.indexOf(self.tabEditDockingStation), _translate("MainWindow", "Docking stations", None))
        self.tabWidget.setTabToolTip(self.tabWidget.indexOf(self.tabEditDockingStation), _translate("MainWindow", "Adjust docking station origin per robot type", None))
        self.menuFile.setTitle(_translate("MainWindow", "&File", None))
        self.actionSave_file.setText(_translate("MainWindow", "&Save as", None))
        self.actionSave_file.setShortcut(_translate("MainWindow", "Ctrl+S", None))
        self.actionLoad_file.setText(_translate("MainWindow", "&Open setup", None))
        self.actionLoad_file.setShortcut(_translate("MainWindow", "Ctrl+O", None))
        self.actionQuit_application.setText(_translate("MainWindow", "&Quit application", None))
        self.actionQuit_application.setShortcut(_translate("MainWindow", "Ctrl+Q", None))
        self.actionNew.setText(_translate("MainWindow", "&New...", None))
        self.actionNew.setShortcut(_translate("MainWindow", "Ctrl+N", None))
        self.actionAbout.setText(_translate("MainWindow", "&About", None))
        self.actionAbout.setShortcut(_translate("MainWindow", "Ctrl+A", None))

import images_rc
