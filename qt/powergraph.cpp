#include "powergraph.h"
#include <cmath>
#include <algorithm>
#include <QFont>

PowerGraph::PowerGraph(QWidget *parent) 
    : QWidget(parent), maxHistory(300), maxPowerKw(210.0) {
    setMinimumSize(300, 130);
}

void PowerGraph::addValue(double kw) {
    history.push_back(kw);
    if (history.size() > maxHistory) {
        history.pop_front();
    }
    update(); // Triggers paintEvent
}

void PowerGraph::setMaxKw(double m) {
    maxPowerKw = m;
}

void PowerGraph::paintEvent(QPaintEvent *event) {
    QPainter painter(this);
    painter.fillRect(rect(), Qt::black);

    int w = width();
    int h = height();
    int midY = h / 2;

    // Draw subtle grid lines (every 50 kW)
    painter.setPen(QPen(QColor(30, 30, 30), 1, Qt::SolidLine));
    for (double kw = 50; kw < maxPowerKw; kw += 50) {
        int yOffset = static_cast<int>((kw / maxPowerKw) * midY);
        painter.drawLine(0, midY - yOffset, w, midY - yOffset); // Positive grid
        painter.drawLine(0, midY + yOffset, w, midY + yOffset); // Negative grid
    }

    // Draw zero reference line with a slight glow
    painter.setPen(QPen(QColor(80, 80, 80), 1, Qt::SolidLine));
    painter.drawLine(0, midY, w, midY);
    if (history.empty()) return;

    double xStep = (double)w / maxHistory;

    painter.setPen(Qt::NoPen); // Use fillRect for cleaner, gap-free bars

    for (size_t i = 0; i < history.size(); ++i) {
        double val = history[i];
        double ratio = val / maxPowerKw;
        
        // Ensure that even small auxiliary loads are visible (min 1px)
        int barHeight = static_cast<int>(ratio * (double)midY);
        if (std::abs(val) > 0.01 && barHeight == 0) {
            barHeight = (val > 0) ? 1 : -1;
        }
        
        if (barHeight == 0) continue;

        // Use floating point for positioning to avoid cumulative integer rounding gaps
        // We add a tiny 0.2px overlap to ensure "black limiters" never appear between bars
        double xPos = (double)w - ((double)(history.size() - i) * xStep);
        double barWidth = xStep + 0.2; 
        
        if (val >= 0) {
            // Consumption: Dynamic Orange to Red
            QLinearGradient grad(xPos, midY - barHeight, xPos, midY);
            if (val > maxPowerKw * 0.8) {
                grad.setColorAt(0, QColor(255, 50, 0));
                grad.setColorAt(1, QColor(150, 20, 0));
            } else {
                grad.setColorAt(0, QColor(255, 140, 0));
                grad.setColorAt(1, QColor(180, 80, 0));
            }
            painter.fillRect(QRectF(xPos, midY - barHeight, barWidth, barHeight), grad);
        } else {
            // Regen: Electric Green
            QLinearGradient grad(xPos, midY, xPos, midY + std::abs(barHeight));
            grad.setColorAt(0, QColor(0, 255, 150));
            grad.setColorAt(1, QColor(0, 100, 60));
            painter.fillRect(QRectF(xPos, midY, barWidth, std::abs(barHeight)), grad);
        }
    }
    
    // Draw a subtle border around the graph
    painter.setPen(QPen(QColor(60, 60, 60), 1));
    painter.drawRect(0, 0, w - 1, h - 1);
}