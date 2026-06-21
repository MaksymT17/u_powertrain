#ifndef SPECVIEWER_H
#define SPECVIEWER_H

#include <QMainWindow>
#include <QVBoxLayout>
#include <QLabel>
#include <QSlider>
#include <QTimer>
#include "powertrain_simulator.h"
#include <QGridLayout>
#include <QComboBox>
#include "powergraph.h"
#include <QCheckBox>
#include <QPainter>
#include <QTimer>
#include <QPushButton>
#include <QGraphicsOpacityEffect>
#include <QPropertyAnimation>

class WindFlowWidget : public QWidget {
    Q_OBJECT
public:
    explicit WindFlowWidget(QWidget *parent = nullptr);
    void setBasePixmap(const QPixmap &pix);
    void setVelocity(double v); // Relative air velocity in m/s
    void updateRotation(double angleDeg);
    
protected:
    void paintEvent(QPaintEvent *event) override;

private:
    QPixmap basePixmap;
    QPixmap rotatedPixmap;
    double velocity = 0.0;
    double angle = 0.0;
    double offset = 0.0;
    QTimer *animTimer; // Timer for animation updates
    struct Line { double x, y, len, speed; };
    QList<Line> lines;
};

class SpecViewer : public QMainWindow {
    Q_OBJECT
public:
    SpecViewer(QWidget *parent = nullptr);
    ~SpecViewer();
private slots:
    void onThrottleChanged(int value);
    void onBrakeChanged(int value);
    void onGradientChanged(int value);
    void onAmbientChanged(int value);
    void onSurfaceChanged(int index);
    void onWindSpeedChanged(int value);
    void onLowBeamToggled(bool checked);
    void onHighBeamToggled(bool checked);
    void onACToggled(bool checked);
    void onACTempChanged(int value);
    void onInfotainmentChanged(int value);
    void onDriveModeChanged(int index);
    void onConfigurationChanged(int index);
    void onWheelConfigChanged(int index);
    void selectRwdConfig();
    void selectAwdConfig();
    void updateSimulation();
    void onIgnitionToggled();
private:
    QVBoxLayout* setupSpecBox(); // Declaration for the helper method
    QVBoxLayout* setupGradientBox();
    QVBoxLayout* setupWindSpeedBox();
    QVBoxLayout* setupSurfaceSelectionBox();
    QVBoxLayout* setupAmbientTempBox();
    QVBoxLayout* setupDriveModeBox();
    QGridLayout* setupTemperatureBox();
    QWidget* setupVisualThermalDisplay();
    QVBoxLayout* setupCabinSystemsBox();
    EVPowertrainSimulator sim;
    QTimer *simTimer;
    PowerGraph *powerGraph;

    QPixmap baseCarPixmap;

    QLabel *speedLabel;
    QLabel *batteryLabel;
    QLabel *distLabel;
    QLabel *timeLabel;
    QLabel *throttleLabel;
    QLabel *brakeLabel;
    QLabel *powerLabel;
    QLabel *statusLabel;
    QSlider *throttleSlider;
    QSlider *brakeSlider;
    QPushButton *ignitionButton;
    
    QCheckBox *lowBeamCheck;
    QCheckBox *highBeamCheck;
    QCheckBox *acCheck;
    QSlider *acTempSlider;
    QLabel *acTempLabel;
    QSlider *infoSlider;

    WindFlowWidget *gradientDisplay;
    QLabel *gradientValueLabel;
    QLabel *gradientDescLabel;
    QSlider *gradientSlider;
    QComboBox *surfaceSelector;
    QComboBox *driveModeSelector;
    QPushButton *rwdButton;
    QPushButton *awdButton;
    QComboBox *wheelSelector;

    QLabel *windLabel;
    QSlider *windSlider;
    QLabel *ambientLabel;
    QSlider *ambientSlider;
    QLabel *motorTempLabel;
    QLabel *inverterTempLabel;
    QLabel *frontMotorTempLabel;
    QLabel *frontInverterTempLabel;
    QLabel *batteryTempLabel;
    QLabel *coolantMILabel;
    QLabel *coolantBatLabel;

    QLabel *coolingRearPTLabel;
    QLabel *coolingFrontPTLabel;
    QLabel *coolingBatteryLabel;

    QLabel *topViewImageLabel;
    QLabel *visualMotorTemp;
    QLabel *visualInverterTemp;
    QLabel *visualFrontMotorTemp;
    QLabel *visualFrontInverterTemp;
    QLabel *visualBatteryTemp;
    QLabel *visualCoolantMILabel;
    QLabel *visualCoolantBatLabel;

    QLabel *specMassLabel;
    QLabel *specTorqueLabel;
    QLabel *specPowerLabel;
    QLabel *specWheelRadiusLabel;
    QLabel *specDragLabel;
    QLabel *specRollResistFrontLabel;
    QLabel *specRollResistRearLabel;

    QGraphicsOpacityEffect *topViewOpacityEffect;
    QPropertyAnimation *topViewFadeAnim;

    QString getPrecheckStatusString(PrecheckStatus status);
    QLabel* addSpecRow(QVBoxLayout *layout, QString name, QString value);
    QString getTempColor(double t, double normalMax, double emergencyMax);
};
#endif