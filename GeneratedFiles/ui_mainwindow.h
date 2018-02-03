/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.7.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFrame>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QScrollArea>
#include <QtWidgets/QSlider>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QToolBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralWidget;
    QPushButton *pushButtonExit;
    QPushButton *pushButtonProc;
    QLabel *labelThresh;
    QLabel *labelHoleR;
    QSlider *sliderThresh;
    QSlider *sliderHoleR;
    QLabel *labelThreshValue;
    QLabel *labelHoleRValue;
    QCheckBox *checkBoxInvert;
    QLabel *labelS2HValue;
    QSlider *sliderS2H;
    QLabel *labelS2H;
    QLabel *labelH2HValue;
    QSlider *sliderH2H;
    QLabel *labelH2H;
    QSpinBox *spinBoxX;
    QSpinBox *spinBoxY;
    QSpinBox *spinBoxL;
    QSpinBox *spinBoxH;
    QLabel *labelX;
    QLabel *labelY;
    QLabel *labelL;
    QLabel *labelH;
    QLabel *labelUnit;
    QTabWidget *tabWidget;
    QWidget *tabRawVideo;
    QScrollArea *scrollArea;
    QWidget *scrollAreaWidgetContents;
    QWidget *tabBwVideo;
    QScrollArea *scrollAreaBw;
    QWidget *scrollAreaWidgetContents_2;
    QScrollArea *scrollArea_3;
    QWidget *scrollAreaWidgetContents_3;
    QWidget *tabResult;
    QScrollArea *scrollAreaResult;
    QWidget *scrollAreaWidgetContents_4;
    QTableWidget *tableResult;
    QLabel *labelROIReal;
    QSpinBox *spinBoxTop;
    QSpinBox *spinBoxButton;
    QSpinBox *spinBoxLeft;
    QSpinBox *spinBoxRight;
    QLabel *labelROIEdge;
    QFrame *line;
    QFrame *line_2;
    QLabel *label;
    QLabel *label_2;
    QLabel *label_3;
    QLabel *label_4;
    QSlider *sliderFrameRate;
    QLabel *labelFrameRate;
    QLabel *labelFrameRateValue;
    QLabel *labelHough;
    QSlider *sliderHoughValue;
    QLabel *labelHoughValue;
    QComboBox *comboBoxAlgoChoose;
    QLabel *labelAlgoTime;
    QLabel *labelAlgoTimeValue;
    QLabel *label_Algotype;
    QMenuBar *menuBar;
    QToolBar *mainToolBar;
    QStatusBar *statusBar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1180, 768);
        MainWindow->setMouseTracking(false);
        centralWidget = new QWidget(MainWindow);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButtonExit = new QPushButton(centralWidget);
        pushButtonExit->setObjectName(QStringLiteral("pushButtonExit"));
        pushButtonExit->setGeometry(QRect(960, 590, 121, 23));
        pushButtonProc = new QPushButton(centralWidget);
        pushButtonProc->setObjectName(QStringLiteral("pushButtonProc"));
        pushButtonProc->setGeometry(QRect(890, 30, 75, 23));
        labelThresh = new QLabel(centralWidget);
        labelThresh->setObjectName(QStringLiteral("labelThresh"));
        labelThresh->setGeometry(QRect(900, 100, 54, 21));
        labelHoleR = new QLabel(centralWidget);
        labelHoleR->setObjectName(QStringLiteral("labelHoleR"));
        labelHoleR->setGeometry(QRect(900, 130, 54, 21));
        sliderThresh = new QSlider(centralWidget);
        sliderThresh->setObjectName(QStringLiteral("sliderThresh"));
        sliderThresh->setGeometry(QRect(940, 100, 160, 22));
        sliderThresh->setMouseTracking(true);
        sliderThresh->setOrientation(Qt::Horizontal);
        sliderThresh->setTickPosition(QSlider::NoTicks);
        sliderHoleR = new QSlider(centralWidget);
        sliderHoleR->setObjectName(QStringLiteral("sliderHoleR"));
        sliderHoleR->setGeometry(QRect(940, 130, 160, 22));
        sliderHoleR->setOrientation(Qt::Horizontal);
        labelThreshValue = new QLabel(centralWidget);
        labelThreshValue->setObjectName(QStringLiteral("labelThreshValue"));
        labelThreshValue->setGeometry(QRect(1110, 100, 54, 21));
        labelHoleRValue = new QLabel(centralWidget);
        labelHoleRValue->setObjectName(QStringLiteral("labelHoleRValue"));
        labelHoleRValue->setGeometry(QRect(1110, 130, 54, 21));
        checkBoxInvert = new QCheckBox(centralWidget);
        checkBoxInvert->setObjectName(QStringLiteral("checkBoxInvert"));
        checkBoxInvert->setGeometry(QRect(1050, 20, 71, 21));
        labelS2HValue = new QLabel(centralWidget);
        labelS2HValue->setObjectName(QStringLiteral("labelS2HValue"));
        labelS2HValue->setGeometry(QRect(1110, 160, 54, 21));
        sliderS2H = new QSlider(centralWidget);
        sliderS2H->setObjectName(QStringLiteral("sliderS2H"));
        sliderS2H->setGeometry(QRect(940, 160, 160, 22));
        sliderS2H->setOrientation(Qt::Horizontal);
        labelS2H = new QLabel(centralWidget);
        labelS2H->setObjectName(QStringLiteral("labelS2H"));
        labelS2H->setGeometry(QRect(890, 160, 54, 21));
        labelH2HValue = new QLabel(centralWidget);
        labelH2HValue->setObjectName(QStringLiteral("labelH2HValue"));
        labelH2HValue->setGeometry(QRect(1110, 190, 54, 21));
        sliderH2H = new QSlider(centralWidget);
        sliderH2H->setObjectName(QStringLiteral("sliderH2H"));
        sliderH2H->setGeometry(QRect(940, 190, 160, 22));
        sliderH2H->setOrientation(Qt::Horizontal);
        labelH2H = new QLabel(centralWidget);
        labelH2H->setObjectName(QStringLiteral("labelH2H"));
        labelH2H->setGeometry(QRect(890, 190, 54, 21));
        spinBoxX = new QSpinBox(centralWidget);
        spinBoxX->setObjectName(QStringLiteral("spinBoxX"));
        spinBoxX->setGeometry(QRect(920, 260, 71, 22));
        spinBoxY = new QSpinBox(centralWidget);
        spinBoxY->setObjectName(QStringLiteral("spinBoxY"));
        spinBoxY->setGeometry(QRect(1050, 260, 71, 22));
        spinBoxL = new QSpinBox(centralWidget);
        spinBoxL->setObjectName(QStringLiteral("spinBoxL"));
        spinBoxL->setGeometry(QRect(920, 310, 71, 22));
        spinBoxH = new QSpinBox(centralWidget);
        spinBoxH->setObjectName(QStringLiteral("spinBoxH"));
        spinBoxH->setGeometry(QRect(1050, 310, 71, 22));
        labelX = new QLabel(centralWidget);
        labelX->setObjectName(QStringLiteral("labelX"));
        labelX->setGeometry(QRect(910, 260, 16, 21));
        labelY = new QLabel(centralWidget);
        labelY->setObjectName(QStringLiteral("labelY"));
        labelY->setGeometry(QRect(1040, 260, 16, 21));
        labelL = new QLabel(centralWidget);
        labelL->setObjectName(QStringLiteral("labelL"));
        labelL->setGeometry(QRect(910, 310, 16, 21));
        labelH = new QLabel(centralWidget);
        labelH->setObjectName(QStringLiteral("labelH"));
        labelH->setGeometry(QRect(1040, 310, 16, 21));
        labelUnit = new QLabel(centralWidget);
        labelUnit->setObjectName(QStringLiteral("labelUnit"));
        labelUnit->setGeometry(QRect(1060, 50, 71, 16));
        QFont font;
        font.setFamily(QStringLiteral("Adobe Arabic"));
        font.setBold(false);
        font.setWeight(50);
        labelUnit->setFont(font);
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(10, 40, 851, 661));
        tabRawVideo = new QWidget();
        tabRawVideo->setObjectName(QStringLiteral("tabRawVideo"));
        scrollArea = new QScrollArea(tabRawVideo);
        scrollArea->setObjectName(QStringLiteral("scrollArea"));
        scrollArea->setGeometry(QRect(0, 0, 851, 641));
        scrollArea->setWidgetResizable(true);
        scrollAreaWidgetContents = new QWidget();
        scrollAreaWidgetContents->setObjectName(QStringLiteral("scrollAreaWidgetContents"));
        scrollAreaWidgetContents->setGeometry(QRect(0, 0, 849, 639));
        scrollArea->setWidget(scrollAreaWidgetContents);
        tabWidget->addTab(tabRawVideo, QString());
        tabBwVideo = new QWidget();
        tabBwVideo->setObjectName(QStringLiteral("tabBwVideo"));
        scrollAreaBw = new QScrollArea(tabBwVideo);
        scrollAreaBw->setObjectName(QStringLiteral("scrollAreaBw"));
        scrollAreaBw->setGeometry(QRect(-10, 0, 861, 641));
        scrollAreaBw->setWidgetResizable(true);
        scrollAreaWidgetContents_2 = new QWidget();
        scrollAreaWidgetContents_2->setObjectName(QStringLiteral("scrollAreaWidgetContents_2"));
        scrollAreaWidgetContents_2->setGeometry(QRect(0, 0, 859, 639));
        scrollArea_3 = new QScrollArea(scrollAreaWidgetContents_2);
        scrollArea_3->setObjectName(QStringLiteral("scrollArea_3"));
        scrollArea_3->setGeometry(QRect(10, 0, 851, 641));
        scrollArea_3->setWidgetResizable(true);
        scrollAreaWidgetContents_3 = new QWidget();
        scrollAreaWidgetContents_3->setObjectName(QStringLiteral("scrollAreaWidgetContents_3"));
        scrollAreaWidgetContents_3->setGeometry(QRect(0, 0, 849, 639));
        scrollArea_3->setWidget(scrollAreaWidgetContents_3);
        scrollAreaBw->setWidget(scrollAreaWidgetContents_2);
        tabWidget->addTab(tabBwVideo, QString());
        tabResult = new QWidget();
        tabResult->setObjectName(QStringLiteral("tabResult"));
        scrollAreaResult = new QScrollArea(tabResult);
        scrollAreaResult->setObjectName(QStringLiteral("scrollAreaResult"));
        scrollAreaResult->setGeometry(QRect(0, 0, 651, 641));
        scrollAreaResult->setWidgetResizable(true);
        scrollAreaWidgetContents_4 = new QWidget();
        scrollAreaWidgetContents_4->setObjectName(QStringLiteral("scrollAreaWidgetContents_4"));
        scrollAreaWidgetContents_4->setGeometry(QRect(0, 0, 649, 639));
        scrollAreaResult->setWidget(scrollAreaWidgetContents_4);
        tableResult = new QTableWidget(tabResult);
        if (tableResult->columnCount() < 2)
            tableResult->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        tableResult->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tableResult->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        tableResult->setObjectName(QStringLiteral("tableResult"));
        tableResult->setGeometry(QRect(650, 0, 201, 641));
        tabWidget->addTab(tabResult, QString());
        labelROIReal = new QLabel(centralWidget);
        labelROIReal->setObjectName(QStringLiteral("labelROIReal"));
        labelROIReal->setGeometry(QRect(890, 230, 81, 21));
        spinBoxTop = new QSpinBox(centralWidget);
        spinBoxTop->setObjectName(QStringLiteral("spinBoxTop"));
        spinBoxTop->setGeometry(QRect(990, 360, 51, 31));
        spinBoxTop->setMaximum(3000);
        spinBoxButton = new QSpinBox(centralWidget);
        spinBoxButton->setObjectName(QStringLiteral("spinBoxButton"));
        spinBoxButton->setGeometry(QRect(990, 480, 51, 31));
        spinBoxButton->setMaximum(3000);
        spinBoxLeft = new QSpinBox(centralWidget);
        spinBoxLeft->setObjectName(QStringLiteral("spinBoxLeft"));
        spinBoxLeft->setGeometry(QRect(910, 420, 51, 31));
        spinBoxLeft->setMaximum(3000);
        spinBoxRight = new QSpinBox(centralWidget);
        spinBoxRight->setObjectName(QStringLiteral("spinBoxRight"));
        spinBoxRight->setGeometry(QRect(1070, 420, 51, 31));
        spinBoxRight->setMaximum(3000);
        labelROIEdge = new QLabel(centralWidget);
        labelROIEdge->setObjectName(QStringLiteral("labelROIEdge"));
        labelROIEdge->setGeometry(QRect(890, 360, 81, 21));
        line = new QFrame(centralWidget);
        line->setObjectName(QStringLiteral("line"));
        line->setGeometry(QRect(890, 340, 241, 20));
        line->setFrameShape(QFrame::HLine);
        line->setFrameShadow(QFrame::Sunken);
        line_2 = new QFrame(centralWidget);
        line_2->setObjectName(QStringLiteral("line_2"));
        line_2->setGeometry(QRect(890, 210, 241, 20));
        line_2->setFrameShape(QFrame::HLine);
        line_2->setFrameShadow(QFrame::Sunken);
        label = new QLabel(centralWidget);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(1000, 390, 41, 31));
        label_2 = new QLabel(centralWidget);
        label_2->setObjectName(QStringLiteral("label_2"));
        label_2->setGeometry(QRect(1000, 450, 51, 31));
        label_3 = new QLabel(centralWidget);
        label_3->setObjectName(QStringLiteral("label_3"));
        label_3->setGeometry(QRect(960, 420, 41, 31));
        label_4 = new QLabel(centralWidget);
        label_4->setObjectName(QStringLiteral("label_4"));
        label_4->setGeometry(QRect(1030, 420, 41, 31));
        sliderFrameRate = new QSlider(centralWidget);
        sliderFrameRate->setObjectName(QStringLiteral("sliderFrameRate"));
        sliderFrameRate->setGeometry(QRect(580, 0, 160, 22));
        sliderFrameRate->setOrientation(Qt::Horizontal);
        labelFrameRate = new QLabel(centralWidget);
        labelFrameRate->setObjectName(QStringLiteral("labelFrameRate"));
        labelFrameRate->setGeometry(QRect(513, 0, 81, 21));
        labelFrameRateValue = new QLabel(centralWidget);
        labelFrameRateValue->setObjectName(QStringLiteral("labelFrameRateValue"));
        labelFrameRateValue->setGeometry(QRect(750, 0, 21, 21));
        labelHough = new QLabel(centralWidget);
        labelHough->setObjectName(QStringLiteral("labelHough"));
        labelHough->setGeometry(QRect(520, 30, 81, 21));
        sliderHoughValue = new QSlider(centralWidget);
        sliderHoughValue->setObjectName(QStringLiteral("sliderHoughValue"));
        sliderHoughValue->setGeometry(QRect(580, 30, 160, 22));
        sliderHoughValue->setMinimum(100);
        sliderHoughValue->setMaximum(180);
        sliderHoughValue->setOrientation(Qt::Horizontal);
        labelHoughValue = new QLabel(centralWidget);
        labelHoughValue->setObjectName(QStringLiteral("labelHoughValue"));
        labelHoughValue->setGeometry(QRect(750, 30, 21, 21));
        comboBoxAlgoChoose = new QComboBox(centralWidget);
        comboBoxAlgoChoose->setObjectName(QStringLiteral("comboBoxAlgoChoose"));
        comboBoxAlgoChoose->setGeometry(QRect(70, 10, 141, 22));
        labelAlgoTime = new QLabel(centralWidget);
        labelAlgoTime->setObjectName(QStringLiteral("labelAlgoTime"));
        labelAlgoTime->setGeometry(QRect(230, 10, 91, 20));
        labelAlgoTimeValue = new QLabel(centralWidget);
        labelAlgoTimeValue->setObjectName(QStringLiteral("labelAlgoTimeValue"));
        labelAlgoTimeValue->setGeometry(QRect(330, 10, 81, 20));
        label_Algotype = new QLabel(centralWidget);
        label_Algotype->setObjectName(QStringLiteral("label_Algotype"));
        label_Algotype->setGeometry(QRect(10, 10, 54, 21));
        MainWindow->setCentralWidget(centralWidget);
        tabWidget->raise();
        pushButtonExit->raise();
        pushButtonProc->raise();
        labelThresh->raise();
        labelHoleR->raise();
        sliderThresh->raise();
        sliderHoleR->raise();
        labelThreshValue->raise();
        labelHoleRValue->raise();
        checkBoxInvert->raise();
        labelS2HValue->raise();
        sliderS2H->raise();
        labelS2H->raise();
        labelH2HValue->raise();
        sliderH2H->raise();
        labelH2H->raise();
        spinBoxX->raise();
        spinBoxY->raise();
        spinBoxL->raise();
        spinBoxH->raise();
        labelX->raise();
        labelY->raise();
        labelL->raise();
        labelH->raise();
        labelUnit->raise();
        labelROIReal->raise();
        spinBoxTop->raise();
        spinBoxButton->raise();
        spinBoxLeft->raise();
        spinBoxRight->raise();
        labelROIEdge->raise();
        line->raise();
        line_2->raise();
        label->raise();
        label_2->raise();
        label_3->raise();
        label_4->raise();
        sliderFrameRate->raise();
        labelFrameRate->raise();
        labelFrameRateValue->raise();
        labelHough->raise();
        sliderHoughValue->raise();
        labelHoughValue->raise();
        comboBoxAlgoChoose->raise();
        labelAlgoTime->raise();
        labelAlgoTimeValue->raise();
        label_Algotype->raise();
        menuBar = new QMenuBar(MainWindow);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1180, 23));
        MainWindow->setMenuBar(menuBar);
        mainToolBar = new QToolBar(MainWindow);
        mainToolBar->setObjectName(QStringLiteral("mainToolBar"));
        MainWindow->addToolBar(Qt::TopToolBarArea, mainToolBar);
        statusBar = new QStatusBar(MainWindow);
        statusBar->setObjectName(QStringLiteral("statusBar"));
        MainWindow->setStatusBar(statusBar);

        retranslateUi(MainWindow);

        tabWidget->setCurrentIndex(0);


        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "\345\206\262\345\216\213\346\216\247\345\210\266", Q_NULLPTR));
        pushButtonExit->setText(QApplication::translate("MainWindow", "\351\200\200\345\207\272", Q_NULLPTR));
        pushButtonProc->setText(QApplication::translate("MainWindow", "\345\206\262\345\216\213\345\244\204\347\220\206", Q_NULLPTR));
        labelThresh->setText(QApplication::translate("MainWindow", "\351\230\210\345\200\274", Q_NULLPTR));
        labelHoleR->setText(QApplication::translate("MainWindow", "\345\255\224\345\276\204", Q_NULLPTR));
        labelThreshValue->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        labelHoleRValue->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        checkBoxInvert->setText(QApplication::translate("MainWindow", "\345\217\215\350\211\262\345\244\204\347\220\206", Q_NULLPTR));
        labelS2HValue->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        labelS2H->setText(QApplication::translate("MainWindow", "\351\232\217\350\276\271\350\267\235\347\246\273", Q_NULLPTR));
        labelH2HValue->setText(QApplication::translate("MainWindow", "0", Q_NULLPTR));
        labelH2H->setText(QApplication::translate("MainWindow", "\345\255\224\351\227\264\350\267\235\347\246\273", Q_NULLPTR));
        labelX->setText(QApplication::translate("MainWindow", "X", Q_NULLPTR));
        labelY->setText(QApplication::translate("MainWindow", "Y", Q_NULLPTR));
        labelL->setText(QApplication::translate("MainWindow", "L", Q_NULLPTR));
        labelH->setText(QApplication::translate("MainWindow", "H", Q_NULLPTR));
        labelUnit->setText(QApplication::translate("MainWindow", "\345\215\225\344\275\215\357\274\232mm", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tabRawVideo), QApplication::translate("MainWindow", "RawVideo", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tabBwVideo), QApplication::translate("MainWindow", "BwVideo", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem = tableResult->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QApplication::translate("MainWindow", "X", Q_NULLPTR));
        QTableWidgetItem *___qtablewidgetitem1 = tableResult->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QApplication::translate("MainWindow", "Y", Q_NULLPTR));
        tabWidget->setTabText(tabWidget->indexOf(tabResult), QApplication::translate("MainWindow", "Result", Q_NULLPTR));
        labelROIReal->setText(QApplication::translate("MainWindow", "ROI\345\256\236\351\231\205\345\260\272\345\257\270", Q_NULLPTR));
        labelROIEdge->setText(QApplication::translate("MainWindow", "ROI\350\276\271\347\225\214", Q_NULLPTR));
        label->setText(QApplication::translate("MainWindow", "\344\270\212\350\276\271\347\225\214", Q_NULLPTR));
        label_2->setText(QApplication::translate("MainWindow", "\344\270\213\351\253\230\345\272\246", Q_NULLPTR));
        label_3->setText(QApplication::translate("MainWindow", "\345\267\246\350\276\271\347\225\214", Q_NULLPTR));
        label_4->setText(QApplication::translate("MainWindow", "\345\217\263\345\256\275\345\272\246", Q_NULLPTR));
        labelFrameRate->setText(QApplication::translate("MainWindow", "\345\270\247\351\227\264\351\232\224(ms)", Q_NULLPTR));
        labelFrameRateValue->setText(QApplication::translate("MainWindow", "30", Q_NULLPTR));
        labelHough->setText(QApplication::translate("MainWindow", "HoughValue", Q_NULLPTR));
        labelHoughValue->setText(QApplication::translate("MainWindow", "150", Q_NULLPTR));
        labelAlgoTime->setText(QApplication::translate("MainWindow", "Algorithm Time: ", Q_NULLPTR));
        labelAlgoTimeValue->setText(QApplication::translate("MainWindow", "0 ms", Q_NULLPTR));
        label_Algotype->setText(QApplication::translate("MainWindow", "\346\216\222\345\255\224\346\226\271\345\274\217", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
