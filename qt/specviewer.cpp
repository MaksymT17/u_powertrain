#include <QApplication>
#include <QIcon>
#include <QPainter>
#include <QPainterPath>
#include <QFont>
#include <QLabel>
#include <QHBoxLayout>
#include <QGroupBox>
#include "specviewer.h"
#include <spdlog/spdlog.h>
#include <cmath>
#include <cstdlib>

WindFlowWidget::WindFlowWidget(QWidget *parent) : QWidget(parent) {
    setFixedSize(310, 130);
    setStyleSheet("background-color: black; border: 1px solid #333;");
    
    animTimer = new QTimer(this);
    connect(animTimer, &QTimer::timeout, this, [this]() {
        if (std::abs(velocity) > 0.1) {
            // Adjust visual displacement speed relative to airflow
            offset += velocity * 0.4;
            update();
        }
    });
    // Aligned with the 50ms global simulation timer to prevent event loop desync
    animTimer->start(50); // ~20 FPS 

    // Generate semi-random flow lines (reproducible seed for visual stability)
    srand(42);
    for (int i = 0; i < 60; ++i) { // Increased pool for higher density for more dynamic scaling
        lines.append({ (double)(rand() % 500 - 100), (double)(rand() % 200 - 35),
                       (double)(20 + rand() % 50), 0.8 + (rand() % 100 / 100.0) });
    }
}

void WindFlowWidget::setBasePixmap(const QPixmap &pix) { 
    basePixmap = pix; 
    updateRotation(0); 
}

void WindFlowWidget::setVelocity(double v) { velocity = v; }

void WindFlowWidget::updateRotation(double angleDeg) {
    angle = angleDeg;
    QTransform trans;
    trans.rotate(angle);
    rotatedPixmap = basePixmap.transformed(trans, Qt::SmoothTransformation);
    update();
}

void WindFlowWidget::paintEvent(QPaintEvent *) {
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);
    
    if (!rotatedPixmap.isNull()) {
        p.drawPixmap((width() - rotatedPixmap.width()) / 2, (height() - rotatedPixmap.height()) / 2, rotatedPixmap);
    }
    // Only draw lines if there's significant airflow
    if (std::abs(velocity) < 0.5) return;

    // Dynamically calculate how many lines to show based on wind strength
    // Base of 8 lines, adding more as velocity increases
    int visibleLineCount = std::clamp(static_cast<int>(8 + std::abs(velocity) * 1.5), 8, static_cast<int>(lines.size()));

    p.save();
    // Rotate the coordinate system around the center to match the car's gradient
    p.translate(width() / 2.0, height() / 2.0);
    p.rotate(angle);
    p.translate(-width() / 2.0, -height() / 2.0);

    // Use a slightly larger virtual width for wrapping to account for rotation tilt
    double virtualWidth = width() + 100.0;

    p.setPen(QPen(QColor(0, 210, 255, 75), 1)); // Cyan, translucent
    for (int i = 0; i < visibleLineCount; ++i) {
        double x = fmod(lines[i].x + offset * lines[i].speed, virtualWidth);
        if (x < -50.0) x += virtualWidth; // Wrap around for continuous animation
        
        p.drawLine(QPointF(x, lines[i].y), QPointF(x + lines[i].len, lines[i].y));
    }
    p.restore();
}

SpecViewer::SpecViewer(QWidget *parent) : QMainWindow(parent) {
    setWindowTitle("EV Prototype - Dashboard [C++]");
    // Sync window icon with the global application icon
    setWindowIcon(qApp->windowIcon()); // Use the globally set squircle icon
    
    setMinimumSize(1280, 860); // Adjusted size for better screen compatibility
    resize(1280, 860);         // Set initial size

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);
    QHBoxLayout *mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setSpacing(15); // Reduced base spacing between major columns

    // Load the car image for the gradient display
    QString imgPath = ":/assets/powertrain_scheme2.png";
    baseCarPixmap = QPixmap(imgPath).scaled(300, 120, Qt::KeepAspectRatio, Qt::SmoothTransformation); // Scaled for the WindFlowWidget

    // --- LEFT COLUMN: Controls & Motion ---
    QString sliderStyle = // Base style for all horizontal sliders
        "QSlider::groove:horizontal { border: 1px solid #333; height: 12px; background: #111; margin: 2px 0; border-radius: 6px; }"
        "QSlider::handle:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #555, stop:1 #222); border: 1px solid #777; width: 18px; margin: -5px 0; border-radius: 9px; }";

    QString throttleHandleStyle = sliderStyle + "QSlider::sub-page:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #332200, stop:1 #ff8800); border-radius: 6px; }";
    QString brakeHandleStyle = sliderStyle + "QSlider::sub-page:horizontal { background: qlineargradient(x1:0, y1:0, x2:1, y2:0, stop:0 #002211, stop:1 #00ff99); border-radius: 6px; }";

    QVBoxLayout *leftCol = new QVBoxLayout();
    leftCol->setContentsMargins(10, 10, 10, 10);
    // Throttle Slider
    throttleLabel = new QLabel("ACCELERATION");
    throttleLabel->setStyleSheet("color: #ff8800; font-weight: bold; font-size: 10px; text-transform: uppercase;");
    throttleSlider = new QSlider(Qt::Horizontal);
    throttleSlider->setRange(0, 100);
    throttleSlider->setStyleSheet(throttleHandleStyle);
    connect(throttleSlider, &QSlider::valueChanged, this, &SpecViewer::onThrottleChanged);
    // Brake Slider
    brakeLabel = new QLabel("REGEN / BRAKING");
    brakeLabel->setStyleSheet("color: #00ff99; font-weight: bold; font-size: 10px; text-transform: uppercase;");
    brakeSlider = new QSlider(Qt::Horizontal);
    brakeSlider->setRange(0, 100);
    brakeSlider->setStyleSheet(brakeHandleStyle);
    connect(brakeSlider, &QSlider::valueChanged, this, &SpecViewer::onBrakeChanged);
    // Speed Label
    speedLabel = new QLabel("0.0 km/h");
    speedLabel->setStyleSheet("font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 42px; font-weight: bold; color: #00ff99;");
    speedLabel->setAlignment(Qt::AlignCenter);

    batteryLabel = new QLabel("Battery: 75.00 kWh");
    distLabel = new QLabel("Distance: 0.000 km");
    timeLabel = new QLabel("Trip Time: 00:00:00");
    coolingRearPTLabel   = new QLabel("Rear Loop: NONE");
    coolingFrontPTLabel  = new QLabel("Front Loop: NONE");
    coolingBatteryLabel  = new QLabel("Bat Loop: NONE");
    // Unified styling for telemetry labels
    // Unified compact styling for the Stats Grid
    QString statStyle = "color: #e0e0e0; font-family: 'Helvetica'; font-size: 12px;";
    batteryLabel->setStyleSheet(statStyle);
    distLabel->setStyleSheet(statStyle);
    timeLabel->setStyleSheet(statStyle);
    coolingRearPTLabel->setStyleSheet(statStyle);
    coolingFrontPTLabel->setStyleSheet(statStyle);
    coolingBatteryLabel->setStyleSheet(statStyle);
    // Power Label
    powerLabel = new QLabel("--- kWh/100km");
    powerLabel->setStyleSheet("font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-weight: bold; color: #00ff99;");
    // Status Label
    statusLabel = new QLabel("✓ System Normal");
    statusLabel->setStyleSheet("font-family: 'Helvetica'; font-size: 15px; font-weight: bold; color: #00ff44;");
    statusLabel->setContentsMargins(0, 2, 0, 5); // Reduced top padding for compactness

    QGroupBox *powerFrame = new QGroupBox("Power Delivery (kW)");
    powerFrame->setStyleSheet("QGroupBox { border: 1px solid #555; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *powerLayout = new QVBoxLayout(powerFrame);

    powerGraph = new PowerGraph(); // Custom PowerGraph widget
    powerLayout->addWidget(powerLabel);
    powerLayout->addWidget(powerGraph);

    leftCol->addWidget(throttleLabel);
    leftCol->addWidget(throttleSlider);
    leftCol->addWidget(brakeLabel);
    leftCol->addWidget(brakeSlider);
    leftCol->addSpacing(20);
    leftCol->addWidget(speedLabel); // Main speed display
    leftCol->addSpacing(5); // Compacted gap after speed

    // --- Stats Grid: Combined Telemetry & Cooling Status ---
    QGroupBox *statsBox = new QGroupBox(""); // Removing title as we are using internal headers
    statsBox->setStyleSheet("QGroupBox { border: 1px solid #333; margin-top: 5px; padding-top: 5px; }");
    QGridLayout *statsLayout = new QGridLayout(statsBox);
    statsLayout->setContentsMargins(10, 5, 10, 10);
    statsLayout->setVerticalSpacing(2);
    statsLayout->setHorizontalSpacing(15);

    // Internal Headers for the stats grid
    QString headerStyle = "color: #555; font-weight: bold; font-size: 10px; text-transform: uppercase;";
    QLabel *statsHeader = new QLabel("Stats");
    statsHeader->setStyleSheet(headerStyle);
    QLabel *coolingHeader = new QLabel("Cooling");
    coolingHeader->setStyleSheet(headerStyle);

    statsLayout->addWidget(statsHeader, 0, 0);
    statsLayout->addWidget(coolingHeader, 0, 2); // Cooling status header

    // Column 0: Telemetry
    statsLayout->addWidget(batteryLabel, 1, 0);
    statsLayout->addWidget(distLabel, 2, 0);
    statsLayout->addWidget(timeLabel, 3, 0); // Trip time

    // Column 1: Vertical Separator (The "Semi-column")
    QFrame *sep = new QFrame();
    sep->setFrameShape(QFrame::VLine);
    sep->setStyleSheet("background-color: #444;");
    sep->setFixedWidth(1);
    statsLayout->addWidget(sep, 0, 1, 4, 1);

    // Column 2: Cooling Status for each component
    statsLayout->addWidget(coolingRearPTLabel, 1, 2);
    statsLayout->addWidget(coolingFrontPTLabel, 2, 2);
    statsLayout->addWidget(coolingBatteryLabel, 3, 2);
    coolingFrontPTLabel->hide(); // Hidden by default (RWD)

    leftCol->addWidget(statsBox);

    leftCol->addWidget(statusLabel);
    leftCol->addWidget(powerFrame);
    leftCol->addLayout(setupCabinSystemsBox());
    leftCol->addStretch();

    // Ignition Button and Pre-check Status
    ignitionButton = new QPushButton("START EV");
    ignitionButton->setStyleSheet("QPushButton { background-color: #00ff44; color: black; font-weight: bold; font-size: 18px; padding: 10px; border-radius: 5px; }"
                                  "QPushButton:hover { background-color: #00cc33; }"
                                  "QPushButton:pressed { background-color: #009922; }");
    connect(ignitionButton, &QPushButton::clicked, this, &SpecViewer::onIgnitionToggled);
    leftCol->addWidget(ignitionButton);

    mainLayout->addLayout(leftCol, 8); 

    // --- MIDDLE COLUMN: Thermal & Environment ---
    QVBoxLayout *midCol = new QVBoxLayout();
    midCol->setContentsMargins(10, 10, 10, 10); // Padding for the middle column
    
    QLabel *thermalHeader = new QLabel("THERMAL MONITOR");
    thermalHeader->setFont(QFont("Verdana", 18, QFont::Bold));
    midCol->addWidget(thermalHeader);

    // --- Consolidated Thermal Monitor Box ---
    QGroupBox *tempBox = new QGroupBox("Thermal Monitor");
    tempBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 20px; }");
    QVBoxLayout *tempBoxLayout = new QVBoxLayout(tempBox);
    tempBoxLayout->setContentsMargins(5, 5, 5, 5);
    tempBoxLayout->setAlignment(Qt::AlignHCenter); // Center the visual thermal display

    QWidget *thermalVisual = setupVisualThermalDisplay();
    tempBoxLayout->addWidget(thermalVisual);
    tempBoxLayout->addLayout(setupTemperatureBox());

    // Align GroupBox width perfectly with the embedded image width plus internal margins
    tempBox->setFixedWidth(thermalVisual->width() + 10);

    midCol->addWidget(tempBox); // Add the thermal monitor box to the middle column
    midCol->addStretch();

    mainLayout->addLayout(midCol, 0); // Stretch 0 forces column to match picture width, reduced spacing
    mainLayout->addSpacing(25);      // Shift third column right

    // --- RIGHT COLUMN: System Info ---
    QVBoxLayout *rightCol = new QVBoxLayout();
    QLabel *header = new QLabel("SYSTEM INFO");
    header->setFont(QFont("Verdana", 18, QFont::Bold));
    rightCol->addWidget(header); // System Info header

    rightCol->addLayout(setupSpecBox());
    rightCol->addLayout(setupSurfaceSelectionBox()); // Now a unified System Configuration box
    rightCol->addLayout(setupGradientBox());
    rightCol->addStretch();

    mainLayout->addLayout(rightCol, 7); // Reduced width by an additional ~10%

    // Setup Simulation Timer (50ms)
    simTimer = new QTimer(this);
    connect(simTimer, &QTimer::timeout, this, &SpecViewer::updateSimulation);
    simTimer->start(50);
}

SpecViewer::~SpecViewer() {
    double dist = sim.getDistanceKm();
    double time = sim.getTripTime();
    double energyConsumed = 75.0 - sim.getBatteryKwh(); // 75.0 is initial capacity

    spdlog::info("--- Trip Summary Report ---");
    spdlog::info("Total Distance:  {:.3f} km", dist);
    spdlog::info("Trip Duration:   {:.0f} seconds", time);
    spdlog::info("Energy Consumed: {:.2f} kWh", energyConsumed);
    spdlog::info("Dashboard closed.");
}

QVBoxLayout* SpecViewer::setupSpecBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *specBox = new QGroupBox("Unit Specifications");
    specBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *specLayout = new QVBoxLayout(specBox);
    // Add static specification rows
    specMassLabel = addSpecRow(specLayout, "MASS", QString("%1 kg").arg(VehicleConstants::RWD_MASS_KG));
    specTorqueLabel = addSpecRow(specLayout, "MAX_WHEEL_TORQUE", QString("%1 Nm").arg(VehicleConstants::REAR_MOTOR_MAX_TORQUE_NM));
    specPowerLabel = addSpecRow(specLayout, "MAX_POWER", QString("%1 W").arg(VehicleConstants::MAX_POWER_W));
    addSpecRow(specLayout, "BATTERY_CAPACITY", QString("%1 kWh").arg(VehicleConstants::BATTERY_CAPACITY_KWH, 0, 'f', 2));
    addSpecRow(specLayout, "EFFICIENCY", "0.88");
    specWheelRadiusLabel = addSpecRow(specLayout, "WHEEL_RADIUS", QString("%1 m").arg(VehicleConstants::WHEEL_RADIUS_21, 0, 'f', 2));
    specDragLabel = addSpecRow(specLayout, "DRAG_COEFF", QString("%1").arg(VehicleConstants::DRAG_COEFF_21, 0, 'f', 3));
    specRollResistFrontLabel = addSpecRow(specLayout, "ROLL_RESIST_FRONT", QString("%1").arg(VehicleConstants::CRR_21_FRONT, 0, 'f', 4));
    specRollResistRearLabel = addSpecRow(specLayout, "ROLL_RESIST_REAR", QString("%1").arg(VehicleConstants::CRR_21_REAR, 0, 'f', 4));
    
    container->addWidget(specBox);
    return container;
}

QVBoxLayout* SpecViewer::setupCabinSystemsBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *infoBox = new QGroupBox("Infotainment & Climate");
    infoBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *layout = new QVBoxLayout(infoBox);

    // Lighting Section
    lowBeamCheck = new QCheckBox("Low Beam (100W)");
    highBeamCheck = new QCheckBox("High Beam (250W)");
    lowBeamCheck->setStyleSheet("color: white;"); // Styling for checkboxes
    highBeamCheck->setStyleSheet("color: white;");
    connect(lowBeamCheck, &QCheckBox::toggled, this, &SpecViewer::onLowBeamToggled);
    connect(highBeamCheck, &QCheckBox::toggled, this, &SpecViewer::onHighBeamToggled);

    // Climate Section
    acCheck = new QCheckBox("Air Conditioner (Auto)");
    acCheck->setStyleSheet("color: #00d1ff; font-weight: bold;");
    connect(acCheck, &QCheckBox::toggled, this, &SpecViewer::onACToggled);

    acTempLabel = new QLabel("Target Temp: 22°C"); // AC target temperature label
    acTempLabel->setStyleSheet("color: #aaa; font-size: 11px;");
    acTempSlider = new QSlider(Qt::Horizontal);
    acTempSlider->setRange(16, 30);
    acTempSlider->setValue(22);
    connect(acTempSlider, &QSlider::valueChanged, this, &SpecViewer::onACTempChanged);

    // Infotainment Section
    QLabel *infoLabel = new QLabel("Computer/Infotainment Load");
    infoLabel->setStyleSheet("color: #aaa; font-size: 11px; margin-top: 5px;"); // Infotainment label
    infoSlider = new QSlider(Qt::Horizontal);
    infoSlider->setRange(50, 500);
    infoSlider->setValue(150);
    connect(infoSlider, &QSlider::valueChanged, this, &SpecViewer::onInfotainmentChanged);

    // Arrangement
    QHBoxLayout *lightsLayout = new QHBoxLayout();
    lightsLayout->addWidget(lowBeamCheck);
    lightsLayout->addWidget(highBeamCheck);
    
    layout->addLayout(lightsLayout);
    layout->addSpacing(10);
    layout->addWidget(acCheck);
    layout->addWidget(acTempLabel);
    layout->addWidget(acTempSlider);
    layout->addSpacing(10);
    layout->addWidget(infoLabel);
    layout->addWidget(infoSlider);
    
    container->addWidget(infoBox);
    return container;
}

QVBoxLayout* SpecViewer::setupGradientBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *gradBox = new QGroupBox("Road Gradient");
    gradBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    
    QHBoxLayout *hLayout = new QHBoxLayout(gradBox); // Layout for gradient box
    
    QVBoxLayout *imageAreaLayout = new QVBoxLayout();
    imageAreaLayout->setSpacing(5);

    gradientSlider = new QSlider(Qt::Vertical);
    gradientSlider->setRange(-20, 20);
    gradientSlider->setValue(0);
    
    // Fancy "Swiper" Styling to match the Python digital dashboard look
    gradientSlider->setStyleSheet(
        "QSlider::groove:vertical {"
        "  background: #111; "
        "  width: 8px; "
        "  border: 1px solid #444;"
        "  border-radius: 4px;"
        "}"
        "QSlider::handle:vertical {"
        "  background: #2255aa; "
        "  border: 1px solid #5588ff; "
        "  height: 20px; "
        "  margin: 0 -6px; "
        "  border-radius: 3px;"
        "}"
    );

    connect(gradientSlider, &QSlider::valueChanged, this, &SpecViewer::onGradientChanged);
    // WindFlowWidget for animated gradient display
    gradientDisplay = new WindFlowWidget();
    gradientDisplay->setBasePixmap(baseCarPixmap);

    gradientDescLabel = new QLabel("FLAT");
    gradientDescLabel->setStyleSheet("font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 13px; font-weight: bold; color: #00d1ff;");

    gradientValueLabel = new QLabel("+0.0%");
    gradientValueLabel->setStyleSheet("font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 14px; font-weight: bold;");
    gradientValueLabel->setFixedWidth(70);
    gradientValueLabel->setAlignment(Qt::AlignCenter);

    QHBoxLayout *labelsLayout = new QHBoxLayout();
    labelsLayout->setSpacing(10);
    labelsLayout->setAlignment(Qt::AlignCenter);
    labelsLayout->addWidget(gradientDescLabel);
    labelsLayout->addWidget(gradientValueLabel);

    imageAreaLayout->addWidget(gradientDisplay);
    imageAreaLayout->addLayout(labelsLayout);

    hLayout->addWidget(gradientSlider);
    hLayout->addLayout(imageAreaLayout);
    
    container->addWidget(gradBox);
    return container;
}

QWidget* SpecViewer::setupVisualThermalDisplay() {
    topViewImageLabel = new QLabel();
    
    topViewOpacityEffect = new QGraphicsOpacityEffect(this);
    topViewImageLabel->setGraphicsEffect(topViewOpacityEffect);
    topViewOpacityEffect->setOpacity(1.0);
    topViewFadeAnim = new QPropertyAnimation(topViewOpacityEffect, "opacity", this);

    QPixmap topView(":/assets/top_view.png");
    // Load and scale the top-view car image
    if (!topView.isNull()) {
        // Scaled to a more compact height for better screen fit
        QPixmap scaled = topView.scaledToHeight(600, Qt::SmoothTransformation);
        topViewImageLabel->setPixmap(scaled);
        topViewImageLabel->setFixedSize(scaled.size());
    } else {
        topViewImageLabel->setFixedSize(280, 600); // Fallback size (adjusted)
        topViewImageLabel->setText("[Top View Missing]");
        topViewImageLabel->setStyleSheet("color: #444; border: 1px dashed #444;");
    }
    topViewImageLabel->setAlignment(Qt::AlignCenter);
    // Base style for visual temperature labels
    QString style = "font-weight: bold; font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 12px; background-color: rgba(10,10,10,220); border: 1px solid #444; border-radius: 2px; padding: 2px;";
    
    // Rear Motor temperature overlay
    visualMotorTemp = new QLabel("Mr: 25.0°C", topViewImageLabel);
    visualMotorTemp->setStyleSheet(style + "color: #00ff99;");
    visualMotorTemp->move(124, 128); 
    // Rear Inverter temperature overlay
    visualInverterTemp = new QLabel("Ir: 25.0°C", topViewImageLabel);
    visualInverterTemp->setStyleSheet(style + "color: #00ff99;");
    visualInverterTemp->move(184, 98); 
    // Battery temperature overlay
    visualBatteryTemp = new QLabel("BAT: 25.0°C", topViewImageLabel);
    visualBatteryTemp->setStyleSheet(style + "color: #00ff99;");
    visualBatteryTemp->move(120, 287); // Center battery pack (adjusted)
    
    // Front Motor temperature overlay
    visualFrontMotorTemp = new QLabel("Mf: 25.0°C", topViewImageLabel);
    visualFrontMotorTemp->setStyleSheet(style + "color: #00ff99;");
    visualFrontMotorTemp->move(124, 479); // Adjusted position
    visualFrontMotorTemp->hide(); // Hidden by default (RWD)
    
    // Front Inverter temperature overlay
    visualFrontInverterTemp = new QLabel("If: 25.0°C", topViewImageLabel);
    visualFrontInverterTemp->setStyleSheet(style + "color: #00ff99;");
    visualFrontInverterTemp->move(184, 496); // Adjusted position
    visualFrontInverterTemp->hide();

    // Coolant battery loop temperature overlay
    visualCoolantBatLabel = new QLabel("B-C: 25.0°C", topViewImageLabel);
    visualCoolantBatLabel->setStyleSheet(style + "color: #00d1ff;");
    visualCoolantBatLabel->move(90, 444); // Adjusted position

    visualCoolantMILabel = new QLabel("M/I-C: 25.0°C", topViewImageLabel);
    visualCoolantMILabel->setStyleSheet(style + "color: #00d1ff;");
    visualCoolantMILabel->move(188, 444); // Adjusted position

    return topViewImageLabel;
}

QVBoxLayout* SpecViewer::setupSurfaceSelectionBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *configBox = new QGroupBox("System Configuration");
    configBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *layout = new QVBoxLayout(configBox);
    layout->setSpacing(8);

    // --- Row 1: Powertrain & Wheels ---
    QHBoxLayout *row1 = new QHBoxLayout();
    QLabel* configLabel = new QLabel("DRIVE:");
    configLabel->setStyleSheet("color: #888; font-size: 10px; font-weight: bold; text-transform: uppercase;");
    
    QHBoxLayout *btnLayout = new QHBoxLayout();
    btnLayout->setSpacing(0);
    rwdButton = new QPushButton("RWD");
    rwdButton->setCursor(Qt::PointingHandCursor);
    awdButton = new QPushButton("AWD");
    awdButton->setCursor(Qt::PointingHandCursor);
    connect(rwdButton, &QPushButton::clicked, this, &SpecViewer::selectRwdConfig);
    connect(awdButton, &QPushButton::clicked, this, &SpecViewer::selectAwdConfig);
    btnLayout->addWidget(rwdButton);
    btnLayout->addWidget(awdButton);

    QLabel* wheelLabel = new QLabel("WHEELS:");
    wheelLabel->setStyleSheet("color: #888; font-size: 10px; font-weight: bold; text-transform: uppercase; margin-left: 10px;");
    wheelSelector = new QComboBox();
    wheelSelector->addItem("18\" Eco");
    wheelSelector->addItem("21\" Perf Staggered");
    wheelSelector->setCurrentIndex(1);
    wheelSelector->setStyleSheet("QComboBox { color: white; background-color: #333; }");
    connect(wheelSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SpecViewer::onWheelConfigChanged);

    row1->addWidget(configLabel);
    row1->addLayout(btnLayout);
    row1->addStretch();
    row1->addWidget(wheelLabel);
    row1->addWidget(wheelSelector);
    layout->addLayout(row1);

    // --- Row 2: Modes & Surface ---
    QHBoxLayout *row2 = new QHBoxLayout();
    driveModeSelector = new QComboBox();
    driveModeSelector->addItem("Economic (50%)");
    driveModeSelector->addItem("Normal (75%)");
    driveModeSelector->addItem("Sport (100%)");
    driveModeSelector->setCurrentIndex(1);
    driveModeSelector->setStyleSheet("QComboBox { color: white; background-color: #333; }");
    connect(driveModeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SpecViewer::onDriveModeChanged);

    surfaceSelector = new QComboBox();
    surfaceSelector->addItem("Asphalt");
    surfaceSelector->addItem("Gravel");
    surfaceSelector->addItem("Ice");
    surfaceSelector->setStyleSheet("QComboBox { color: white; background-color: #333; }");
    connect(surfaceSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &SpecViewer::onSurfaceChanged);

    row2->addWidget(new QLabel("MODE:"));
    row2->addWidget(driveModeSelector);
    row2->addSpacing(10);
    row2->addWidget(new QLabel("SURFACE:"));
    row2->addWidget(surfaceSelector);
    layout->addLayout(row2);

    // --- Row 3: Ambient ---
    ambientLabel = new QLabel("Ambient: 25.0°C");
    ambientLabel->setStyleSheet("color: #e0e0e0; font-family: 'Helvetica'; font-size: 13px;");
    ambientSlider = new QSlider(Qt::Horizontal);
    ambientSlider->setRange(-20, 50);
    ambientSlider->setValue(25);
    ambientSlider->setStyleSheet("height: 20px;");
    connect(ambientSlider, &QSlider::valueChanged, this, &SpecViewer::onAmbientChanged);
    layout->addWidget(ambientLabel);
    layout->addWidget(ambientSlider);

    // --- Row 4: Wind ---
    windLabel = new QLabel("Wind: 0 m/s (Calm)");
    windLabel->setStyleSheet("color: #e0e0e0; font-family: 'Helvetica'; font-size: 13px;");
    windSlider = new QSlider(Qt::Horizontal);
    windSlider->setRange(-20, 20);
    windSlider->setValue(0);
    windSlider->setStyleSheet("height: 20px;");
    connect(windSlider, &QSlider::valueChanged, this, &SpecViewer::onWindSpeedChanged);
    layout->addWidget(windLabel);
    layout->addWidget(windSlider);

    selectRwdConfig();
    container->addWidget(configBox);
    return container;
}

QVBoxLayout* SpecViewer::setupDriveModeBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *modeBox = new QGroupBox("Driving Mode");
    modeBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *layout = new QVBoxLayout(modeBox);
    // Drive mode selector
    driveModeSelector = new QComboBox();
    driveModeSelector->addItem("Economic (50%)");
    driveModeSelector->addItem("Normal (75%)");
    driveModeSelector->addItem("Sport (100%)");
    driveModeSelector->setCurrentIndex(1); // Start in Normal
    driveModeSelector->setStyleSheet("QComboBox { color: white; background-color: #333; }");
    
    connect(driveModeSelector, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SpecViewer::onDriveModeChanged);

    layout->addWidget(driveModeSelector);
    container->addWidget(modeBox);
    return container;
}

QVBoxLayout* SpecViewer::setupWindSpeedBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *windBox = new QGroupBox("Wind Speed Selector");
    windBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *layout = new QVBoxLayout(windBox);
    // Wind speed label
    windLabel = new QLabel("Wind: 0 m/s (Calm)");
    windLabel->setStyleSheet("color: #e0e0e0; font-family: 'Helvetica'; font-size: 13px;");
    // Wind speed slider
    windSlider = new QSlider(Qt::Horizontal);
    windSlider->setRange(-20, 20); // Range from headwind to tailwind
    windSlider->setValue(0);
    windSlider->setTickPosition(QSlider::TicksBelow);
    windSlider->setTickInterval(5);

    connect(windSlider, &QSlider::valueChanged, this, &SpecViewer::onWindSpeedChanged);

    layout->addWidget(windLabel);
    layout->addWidget(windSlider);
    container->addWidget(windBox);
    return container;
}

QVBoxLayout* SpecViewer::setupAmbientTempBox() {
    QVBoxLayout *container = new QVBoxLayout();
    QGroupBox *ambBox = new QGroupBox("Ambient Temperature");
    ambBox->setStyleSheet("QGroupBox { border: 1px solid #00d1ff; color: #00d1ff; font-weight: bold; margin-top: 10px; padding-top: 15px; }");
    QVBoxLayout *layout = new QVBoxLayout(ambBox);
    // Ambient temperature label
    ambientLabel = new QLabel("Ambient: 25.0°C");
    ambientLabel->setStyleSheet("color: #e0e0e0; font-family: 'Helvetica'; font-size: 13px;");
    // Ambient temperature slider
    ambientSlider = new QSlider(Qt::Horizontal);
    ambientSlider->setRange(-20, 50);
    ambientSlider->setValue(25);
    connect(ambientSlider, &QSlider::valueChanged, this, &SpecViewer::onAmbientChanged);

    layout->addWidget(ambientLabel);
    layout->addWidget(ambientSlider);
    container->addWidget(ambBox);
    return container;
}

QGridLayout* SpecViewer::setupTemperatureBox() {
    QGridLayout *grid = new QGridLayout();
    grid->setHorizontalSpacing(15);
    grid->setColumnMinimumWidth(0, 135); // Fit "Coolant M/I: 00.0°C"
    grid->setColumnMinimumWidth(1, 125); // Fit "Inverter: 00.0°C"
    QString style = "font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 14px; font-weight: bold; background-color: black;"; // Styling for temperature labels

    coolantMILabel = new QLabel("Coolant M/I: 25.0°C");
    motorTempLabel = new QLabel("Motor:       25.0°C");
    inverterTempLabel = new QLabel("Inverter:    25.0°C");
    frontMotorTempLabel = new QLabel("Front Motor: 25.0°C");
    frontInverterTempLabel = new QLabel("Front Inv:   25.0°C");
    coolantBatLabel = new QLabel("Coolant Bat: 25.0°C");
    batteryTempLabel = new QLabel("Battery:     25.0°C");

    coolantMILabel->setStyleSheet(style);
    motorTempLabel->setStyleSheet(style);
    inverterTempLabel->setStyleSheet(style);
    frontMotorTempLabel->setStyleSheet(style);
    frontInverterTempLabel->setStyleSheet(style);
    coolantBatLabel->setStyleSheet(style);
    batteryTempLabel->setStyleSheet(style);

    // Replicating the logical grid grouping from Python
    grid->addWidget(coolantMILabel,    0, 0, Qt::AlignLeft);
    grid->addWidget(motorTempLabel,    0, 1, Qt::AlignLeft);
    grid->addWidget(inverterTempLabel, 1, 1, Qt::AlignLeft);
    
    grid->addWidget(frontMotorTempLabel, 2, 1, Qt::AlignLeft);
    grid->addWidget(frontInverterTempLabel, 3, 1, Qt::AlignLeft);

    // Vertical spacing between groups
    grid->setRowMinimumHeight(4, 15);

    grid->addWidget(coolantBatLabel,   5, 0, Qt::AlignLeft);
    grid->addWidget(batteryTempLabel,  5, 1, Qt::AlignLeft);

    frontMotorTempLabel->hide();
    frontInverterTempLabel->hide();

    return grid;
}

QLabel* SpecViewer::addSpecRow(QVBoxLayout *layout, QString name, QString value) {
    QLabel *label = new QLabel(QString("%1: %2").arg(name).arg(value));
    label->setStyleSheet("color: #888888; font-family: 'Helvetica'; font-size: 13px;");
    layout->addWidget(label); // Add a specification row to the layout
    return label;
}

void SpecViewer::onThrottleChanged(int value) {
    double val = value / 100.0;
    sim.setThrottle(val);
    if (value > 0) spdlog::info("User Input: Throttle pedal pushed to {}%", value);
    if (value % 10 == 0 || value == 100) spdlog::debug("Input: Throttle set to {}%", value);
    throttleLabel->setText(QString("Throttle: %1%").arg(value));
    if (val > 0) {
        brakeSlider->setValue(0);
        sim.setBrake(0);
    } // If throttle is applied, release brake
}

void SpecViewer::onBrakeChanged(int value) {
    double val = value / 100.0;
    sim.setBrake(val);
    if (value > 0) spdlog::info("User Input: Brake pedal pushed to {}%", value);
    if (value > 0) spdlog::debug("Input: Brake set to {}%", value);
    brakeLabel->setText(QString("Brake: %1%").arg(value));
    if (val > 0) {
        throttleSlider->setValue(0);
        sim.setThrottle(0);
    } // If brake is applied, release throttle
}

void SpecViewer::onGradientChanged(int value) {
    double pct = (double)value;
    double angleDeg = std::atan(pct / 100.0) * (180.0 / M_PI);
    sim.setGradient(angleDeg);
    spdlog::info("Environment: Road Gradient changed to {}%", pct);

    gradientDisplay->updateRotation(angleDeg);

    QString desc;
    if (pct > 0.1)       desc = "UPHILL";
    else if (pct < -0.1) desc = "DOWNHILL";
    else                 desc = "FLAT"; // Determine gradient description

    gradientDescLabel->setText(desc);

    // Format percentage to fixed width (e.g., "+20.0", " +5.0", "-20.0")
    QString pctStr = QString::asprintf("%+5.1f", pct); // Ensures fixed width of 5 chars, including sign
    gradientValueLabel->setText(QString("%1%").arg(pctStr));
}

void SpecViewer::onAmbientChanged(int value) {
    sim.setAmbientTemp((double)value);
    spdlog::info("Environment: Ambient Temp set to {} C", value);
    ambientLabel->setText(QString("Ambient: %1.0°C").arg(value));
}

void SpecViewer::onSurfaceChanged(int index) {
    sim.setSurfaceType(index);
}

void SpecViewer::onWindSpeedChanged(int value) {
    sim.setWindSpeed((double)value);
    QString desc = (value == 0) ? "Calm" : (value > 0 ? "Tailwind" : "Headwind");
    windLabel->setText(QString("Wind: %1 m/s (%2)").arg(value).arg(desc)); // Update wind speed label
    spdlog::info("Environment: Wind Speed set to {} m/s ({})", value, desc.toStdString());
}

void SpecViewer::onLowBeamToggled(bool checked) {
    sim.setLowBeams(checked);
    spdlog::info("Cabin: Low Beams {}", checked ? "ON" : "OFF");
    if (checked) highBeamCheck->setChecked(false);
    } // Ensure only one beam type is active

void SpecViewer::onHighBeamToggled(bool checked) {
    sim.setHighBeams(checked);
    spdlog::info("Cabin: High Beams {}", checked ? "ON" : "OFF");
    if (checked) lowBeamCheck->setChecked(false);
}

void SpecViewer::onACToggled(bool checked) {
    sim.setACOn(checked);
    spdlog::info("Cabin: Air Conditioning {}", checked ? "Enabled" : "Disabled");
    } // Toggle AC

void SpecViewer::onACTempChanged(int value) {
    sim.setACTargetTemp((double)value);
    acTempLabel->setText(QString("Target Temp: %1°C").arg(value));
}

void SpecViewer::onInfotainmentChanged(int value) {
    sim.setInfotainmentPower((double)value);
}

void SpecViewer::onDriveModeChanged(int index) {
    sim.setDriveMode(index);
}

void SpecViewer::onWheelConfigChanged(int index) {
    sim.setWheelConfig(index);
    
    if (index == 0) { // Eco 18"
        specWheelRadiusLabel->setText(QString("WHEEL_RADIUS: %1 m").arg(VehicleConstants::WHEEL_RADIUS_18, 0, 'f', 2));
        specDragLabel->setText(QString("DRAG_COEFF: %1").arg(VehicleConstants::DRAG_COEFF_18, 0, 'f', 3));
        specRollResistFrontLabel->setText(QString("ROLL_RESIST_FRONT: %1").arg(VehicleConstants::CRR_18_FRONT, 0, 'f', 4));
        specRollResistRearLabel->setText(QString("ROLL_RESIST_REAR: %1").arg(VehicleConstants::CRR_18_REAR, 0, 'f', 4));
    } else { // Staggered 21"
        specWheelRadiusLabel->setText(QString("WHEEL_RADIUS: %1 m").arg(VehicleConstants::WHEEL_RADIUS_21, 0, 'f', 2));
        specDragLabel->setText(QString("DRAG_COEFF: %1").arg(VehicleConstants::DRAG_COEFF_21, 0, 'f', 3));
        specRollResistFrontLabel->setText(QString("ROLL_RESIST_FRONT: %1").arg(VehicleConstants::CRR_21_FRONT, 0, 'f', 4));
        specRollResistRearLabel->setText(QString("ROLL_RESIST_REAR: %1").arg(VehicleConstants::CRR_21_REAR, 0, 'f', 4));
    }
}

void SpecViewer::selectRwdConfig() {
    QString rwdStyle = "QPushButton { background-color: #00d1ff; color: black; border: 1px solid #00d1ff; font-weight: bold; padding: 5px; border-top-left-radius: 4px; border-bottom-left-radius: 4px; border-right: none; }"
                       "QPushButton:disabled { background-color: #005566; color: #333; border-color: #005566; }";
    QString awdStyle = "QPushButton { background-color: #222; color: #888; border: 1px solid #444; font-weight: bold; padding: 5px; border-top-right-radius: 4px; border-bottom-right-radius: 4px; }"
                       "QPushButton:disabled { background-color: #111; color: #444; border-color: #222; }";
    rwdButton->setStyleSheet(rwdStyle);
    awdButton->setStyleSheet(awdStyle);
    onConfigurationChanged(0);
}

void SpecViewer::selectAwdConfig() {
    QString rwdStyle = "QPushButton { background-color: #222; color: #888; border: 1px solid #444; font-weight: bold; padding: 5px; border-top-left-radius: 4px; border-bottom-left-radius: 4px; border-right: none; }"
                       "QPushButton:disabled { background-color: #111; color: #444; border-color: #222; }";
    QString awdStyle = "QPushButton { background-color: #00d1ff; color: black; border: 1px solid #00d1ff; font-weight: bold; padding: 5px; border-top-right-radius: 4px; border-bottom-right-radius: 4px; }"
                       "QPushButton:disabled { background-color: #005566; color: #333; border-color: #005566; }";
    rwdButton->setStyleSheet(rwdStyle);
    awdButton->setStyleSheet(awdStyle);
    onConfigurationChanged(1);
}

// Helper function to get color based on temperature thresholds

void SpecViewer::onConfigurationChanged(int index) {
    sim.setConfiguration(index);
    
    // Update Top View Image with Smooth Fade Effect
    QString imgPath = (index == 1) ? ":/assets/top_view_awd.jpg" : ":/assets/top_view.png";
    
    topViewFadeAnim->stop(); // Interrupt any currently running transitions safely
    topViewFadeAnim->disconnect();
    topViewFadeAnim->setDuration(200); // 200ms Fade Out
    topViewFadeAnim->setStartValue(topViewOpacityEffect->opacity());
    topViewFadeAnim->setEndValue(0.0);
    
    connect(topViewFadeAnim, &QPropertyAnimation::finished, this, [this, index, imgPath]() {
        QPixmap topView(imgPath);
        
        // Handle UI label visibility mid-fade for the smoothest visual pop
        if (index == 1) { // AWD Selected
            visualFrontMotorTemp->show();
            visualFrontInverterTemp->show();
            frontMotorTempLabel->show();
            frontInverterTempLabel->show();
            coolingFrontPTLabel->show();
        } else {
            visualFrontMotorTemp->hide();
            visualFrontInverterTemp->hide();
            frontMotorTempLabel->hide();
            frontInverterTempLabel->hide();
            coolingFrontPTLabel->hide();
        }

        // Fallback to direct absolute path to bypass CMake/QRC caching issues
        if (topView.isNull()) {
            QString fallbackPath = (index == 1) ? "/Users/mba23/projects/ucar_helpers/powertrain/top_view_awd.jpg" 
                                                : "/Users/mba23/projects/ucar_helpers/powertrain/top_view.png";
            topView.load(fallbackPath);
        }

        if (!topView.isNull()) {
            topViewImageLabel->setPixmap(topView.scaledToHeight(600, Qt::SmoothTransformation));
        } else {
            spdlog::error("UI Error: Could not load {}. Make sure it is registered in your .qrc file!", imgPath.toStdString());
            topViewImageLabel->setText("[Image Missing: " + imgPath + "]");
            topViewImageLabel->setStyleSheet("color: red; border: 1px dashed red;");
        }
        topViewFadeAnim->disconnect();
        topViewFadeAnim->setDuration(200); // 200ms Fade In
        topViewFadeAnim->setStartValue(0.0);
        topViewFadeAnim->setEndValue(1.0);
        topViewFadeAnim->start();
    });
    topViewFadeAnim->start();

    // Update Specs depending on the selected configuration
    if (index == 1) { // AWD Configuration (Adds Front ACIM)
        specMassLabel->setText(QString("MASS: %1 kg").arg(VehicleConstants::AWD_MASS_KG));
        specTorqueLabel->setText(QString("MAX_WHEEL_TORQUE: %1 Nm").arg(VehicleConstants::REAR_MOTOR_MAX_TORQUE_NM + VehicleConstants::FRONT_MOTOR_MAX_TORQUE_NM));
        specPowerLabel->setText(QString("MAX_POWER: %1 W").arg(VehicleConstants::MAX_AWD_DISPLAY_POWER_W));
    } else {          // RWD Configuration (Current)
        specMassLabel->setText(QString("MASS: %1 kg").arg(VehicleConstants::RWD_MASS_KG));
        specTorqueLabel->setText(QString("MAX_WHEEL_TORQUE: %1 Nm").arg(VehicleConstants::REAR_MOTOR_MAX_TORQUE_NM));
        specPowerLabel->setText(QString("MAX_POWER: %1 W").arg(VehicleConstants::MAX_POWER_W));
    }
}

QString SpecViewer::getPrecheckStatusString(PrecheckStatus status) {
    switch (status) {
        case PrecheckStatus::Initializing: return "Initializing...";
        case PrecheckStatus::HeatingBattery: return "Heating Battery...";
        case PrecheckStatus::CoolingBattery: return "Cooling Battery...";
        case PrecheckStatus::HeatingPT: return "Heating Powertrain...";
        case PrecheckStatus::CoolingPT: return "Cooling Powertrain...";
        case PrecheckStatus::Ready: return "Ready to Drive";
        default: return "Unknown Status";
    }
}
QString SpecViewer::getTempColor(double t, double normalMax, double emergencyMax) {
    if (t > emergencyMax) return "color: red;";
    if (t > normalMax)    return "color: orange;";
    return "color: #00ff99;";
}

void SpecViewer::updateSimulation() {
    sim.update(0.05); // Fixed DT 50ms

    // Disable throttle/brake if ignition is off or pre-checks not ready
    throttleSlider->setEnabled(sim.isIgnitionOn() && sim.getPrecheckStatus() == PrecheckStatus::Ready);
    brakeSlider->setEnabled(sim.isIgnitionOn() && sim.getPrecheckStatus() == PrecheckStatus::Ready);

    // Sync UI sliders with simulator state (handles auto-reset at stop)
    int currentSimThrottle = static_cast<int>(sim.getThrottle() * 100.0);
    int currentSimBrake = static_cast<int>(sim.getBrake() * 100.0);
    if (throttleSlider->value() != currentSimThrottle) throttleSlider->setValue(currentSimThrottle);
    if (brakeSlider->value() != currentSimBrake)       brakeSlider->setValue(currentSimBrake);
    
    // Lock Configuration once the vehicle has moved
    bool hasMoved = sim.getDistanceKm() > 0.001;
    if (rwdButton->isEnabled() == hasMoved) {
        rwdButton->setEnabled(!hasMoved);
        awdButton->setEnabled(!hasMoved);
        rwdButton->setCursor(hasMoved ? Qt::ForbiddenCursor : Qt::PointingHandCursor);
        awdButton->setCursor(hasMoved ? Qt::ForbiddenCursor : Qt::PointingHandCursor);
    }

    // --- PERFORMANCE OPTIMIZATION: State Caching ---
    // Only allocate new strings and update UI if values have noticeably changed
    static double lastSpeed = -1.0;
    if (std::abs(sim.getSpeedKmh() - lastSpeed) > 0.05) {
        lastSpeed = sim.getSpeedKmh();
        speedLabel->setText(QString("%1 km/h").arg(lastSpeed, 0, 'f', 1));
    }

    static double lastBat = -1.0;
    if (std::abs(sim.getBatteryKwh() - lastBat) > 0.01) {
        lastBat = sim.getBatteryKwh();
        batteryLabel->setText(QString("Battery: %1 kWh (%2%)").arg(lastBat, 0, 'f', 2).arg(sim.getSOC(), 0, 'f', 1));
    }

    static double lastDist = -1.0;
    if (std::abs(sim.getDistanceKm() - lastDist) > 0.001) {
        lastDist = sim.getDistanceKm();
        distLabel->setText(QString("Distance: %1 km").arg(lastDist, 0, 'f', 3));
    }
    
    // Update wind flow animation based on relative velocity
    double v_rel = (sim.getSpeedKmh() / 3.6) - windSlider->value();
    gradientDisplay->setVelocity(v_rel);

    // Feed real-time data to the graph
    powerGraph->addValue(sim.getPowerKw());

    // Update power and efficiency labels based on simulation data
    double currentKw = sim.getPowerKw();
    double eff = sim.getEfficiency();
    
    if (eff > 0.0) {
        powerLabel->setText(QString("%1 kW   •   %2 kWh/100km").arg(currentKw, 6, 'f', 2).arg(eff, 0, 'f', 1));
    } else {
        powerLabel->setText(QString("%1 kW   •   --- kWh/100km").arg(currentKw, 6, 'f', 2));
    }

    int t = (int)sim.getTripTime();
    timeLabel->setText(QString("Trip Time: %1:%2:%3").arg(t/3600, 2, 10, QChar('0')).arg((t%3600)/60, 2, 10, QChar('0')).arg(t%60, 2, 10, QChar('0')));
    // Update overall status message and styling
    // Update Overall Status
    QString modeName;
    switch(sim.getDriveMode()) {
        case DriveMode::ECONOMIC: modeName = "ECO"; break;
        case DriveMode::NORMAL:   modeName = "NORMAL"; break;
        case DriveMode::SPORT:    modeName = "SPORT"; break;
    }

    // Update Ignition Button and Pre-check Status
    if (sim.isIgnitionOn()) {
        ignitionButton->setText("STOP EV");
        ignitionButton->setStyleSheet("QPushButton { background-color: #ff4400; color: black; font-weight: bold; font-size: 18px; padding: 10px; border-radius: 5px; }"
                                      "QPushButton:hover { background-color: #cc3300; }"
                                      "QPushButton:pressed { background-color: #992200; }");
    } else {
        ignitionButton->setText("START EV");
        ignitionButton->setStyleSheet("QPushButton { background-color: #00ff44; color: black; font-weight: bold; font-size: 18px; padding: 10px; border-radius: 5px; }"
                                      "QPushButton:hover { background-color: #00cc33; }"
                                      "QPushButton:pressed { background-color: #009922; }");
    }

    // --- Unified System Status (Lexus/Mercedes Perception) ---
    QString fullStatus;
    QString statusColor = "#00ff44"; // Default Green

    if (!sim.isIgnitionOn()) {
        fullStatus = "SYSTEM STANDBY • PRESS START";
        statusColor = "#666666"; // Dim grey for standby
    } else if (sim.getPrecheckStatus() != PrecheckStatus::Ready) {
        fullStatus = "CONDITIONING • " + getPrecheckStatusString(sim.getPrecheckStatus()).toUpper();
        statusColor = "orange";
    } else if (sim.isEmergency()) {
        if (sim.getSpeedKmh() < 0.1) fullStatus = "⚠ TEMP RESTRICTION • OUTSIDE LIMITS";
        else                    fullStatus = "⚠ EMERGENCY • LIMP MODE 20%";
        statusColor = "red";
    } else if (sim.isDerated()) {
        fullStatus = "⚡ " + modeName + " • THERMAL DERATE 70%";
        statusColor = "orange";
    } else {
        fullStatus = "✓ " + modeName + " • SYSTEM NORMAL";
        statusColor = "#00ff44";
    }

    statusLabel->setText(fullStatus);
    statusLabel->setStyleSheet(QString("font-family: 'Helvetica'; font-size: 15px; font-weight: bold; color: %1;").arg(statusColor));

    // Update Thermal Labels with color coding
    QString baseStyle = "font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 14px; font-weight: bold; background-color: black;";
    
    coolantMILabel->setText(QString("Coolant M/I: %1°C").arg(sim.getCoolantPTTemp(), 5, 'f', 1));
    
    motorTempLabel->setText(QString("Motor:       %1°C").arg(sim.getMotorTemp(), 5, 'f', 1));
    motorTempLabel->setStyleSheet(baseStyle + getTempColor(sim.getMotorTemp(), VehicleConstants::MOTOR_NORMAL_MAX, VehicleConstants::MOTOR_CRITICAL));
    
    inverterTempLabel->setText(QString("Inverter:    %1°C").arg(sim.getInverterTemp(), 5, 'f', 1));
    inverterTempLabel->setStyleSheet(baseStyle + getTempColor(sim.getInverterTemp(), VehicleConstants::INV_NORMAL_MAX, VehicleConstants::INV_CRITICAL));
    
    frontMotorTempLabel->setText(QString("Front Motor: %1°C").arg(sim.getFrontMotorTemp(), 5, 'f', 1));
    frontMotorTempLabel->setStyleSheet(baseStyle + getTempColor(sim.getFrontMotorTemp(), VehicleConstants::MOTOR_NORMAL_MAX, VehicleConstants::MOTOR_CRITICAL));

    frontInverterTempLabel->setText(QString("Front Inv:   %1°C").arg(sim.getFrontInverterTemp(), 5, 'f', 1));
    frontInverterTempLabel->setStyleSheet(baseStyle + getTempColor(sim.getFrontInverterTemp(), VehicleConstants::INV_NORMAL_MAX, VehicleConstants::INV_CRITICAL));

    coolantBatLabel->setText(QString("Coolant Bat: %1°C").arg(sim.getCoolantBatTemp(), 5, 'f', 1));
    
    // Update Visual Overlays with color-coded temperatures
    QString vStyle = "font-weight: bold; font-family: 'Menlo', 'Monaco', 'Consolas', 'Courier New', monospace; font-size: 12px; background-color: rgba(10,10,10,220); border: 1px solid #444; border-radius: 2px; padding: 2px;";
    
    visualMotorTemp->setText(QString("Mr: %1°C").arg(sim.getMotorTemp(), 4, 'f', 1));
    visualMotorTemp->setStyleSheet(vStyle + getTempColor(sim.getMotorTemp(), VehicleConstants::MOTOR_NORMAL_MAX, VehicleConstants::MOTOR_CRITICAL));

    visualInverterTemp->setText(QString("Ir: %1°C").arg(sim.getInverterTemp(), 4, 'f', 1));
    visualInverterTemp->setStyleSheet(vStyle + getTempColor(sim.getInverterTemp(), VehicleConstants::INV_NORMAL_MAX, VehicleConstants::INV_CRITICAL));

    if (sim.getConfiguration() == PowertrainConfig::AWD) {
        visualFrontMotorTemp->setText(QString("Mf: %1°C").arg(sim.getFrontMotorTemp(), 4, 'f', 1));
        visualFrontMotorTemp->setStyleSheet(vStyle + getTempColor(sim.getFrontMotorTemp(), VehicleConstants::MOTOR_NORMAL_MAX, VehicleConstants::MOTOR_CRITICAL));
        
        visualFrontInverterTemp->setText(QString("If: %1°C").arg(sim.getFrontInverterTemp(), 4, 'f', 1));
        visualFrontInverterTemp->setStyleSheet(vStyle + getTempColor(sim.getFrontInverterTemp(), VehicleConstants::INV_NORMAL_MAX, VehicleConstants::INV_CRITICAL));
    }

    visualBatteryTemp->setText(QString("BAT: %1°C").arg(sim.getBatteryTemp(), 4, 'f', 1));
    visualBatteryTemp->setStyleSheet(vStyle + getTempColor(sim.getBatteryTemp(), VehicleConstants::BAT_NORMAL_MAX, VehicleConstants::BAT_CRITICAL));

    visualCoolantMILabel->setText(QString("M/I-C: %1°C").arg(sim.getCoolantPTTemp(), 4, 'f', 1));
    visualCoolantBatLabel->setText(QString("B-C: %1°C").arg(sim.getCoolantBatTemp(), 4, 'f', 1));

    batteryTempLabel->setText(QString("Battery:     %1°C").arg(sim.getBatteryTemp(), 5, 'f', 1));
    batteryTempLabel->setStyleSheet(baseStyle + getTempColor(sim.getBatteryTemp(), VehicleConstants::BAT_NORMAL_MAX, VehicleConstants::BAT_CRITICAL));

    // Update Cooling Status Labels using a lambda for string conversion from CoolingAction enum
    auto actionToString = [](CoolingAction action) -> QString {
        switch (action) {
            case CoolingAction::LIQUID_WARM: return "LIQUID WARM";
            case CoolingAction::LIQUID_LOW:  return "LIQUID LOW";
            case CoolingAction::LIQUID_MED:  return "LIQUID MED";
            case CoolingAction::LIQUID_HIGH: return "LIQUID HIGH";
            case CoolingAction::TURNED_OFF:  return "TURNED OFF";
            default:                         return "UNKNOWN"; // Fallback for safety
        }
    };

    // Pump Logic: The loop's pump speed is dictated by the component that is hottest/most in need
    auto getLoopAction = [](CoolingAction a, CoolingAction b) -> CoolingAction {
        if (a == CoolingAction::LIQUID_WARM || b == CoolingAction::LIQUID_WARM) {
            if (a > CoolingAction::TURNED_OFF) return a; // Active cooling takes priority over warm-up
            if (b > CoolingAction::TURNED_OFF) return b;
            return CoolingAction::LIQUID_WARM;
        }
        return (a > b) ? a : b;
    };

    coolingRearPTLabel->setText(QString("Rear Loop:  %1").arg(actionToString(getLoopAction(sim.getMotorAction(), sim.getInverterAction()))));
    coolingFrontPTLabel->setText(QString("Front Loop: %1").arg(actionToString(getLoopAction(sim.getFrontMotorAction(), sim.getFrontInverterAction()))));
    coolingBatteryLabel->setText(QString("Bat Loop:   %1").arg(actionToString(sim.getBatteryAction())));
}

void SpecViewer::onIgnitionToggled() {
    sim.setIgnition(!sim.isIgnitionOn());
}
