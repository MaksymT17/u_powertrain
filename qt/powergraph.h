#ifndef POWERGRAPH_H
#define POWERGRAPH_H

#include <QWidget>
#include <QPainter>
#include <deque>

class PowerGraph : public QWidget {
    Q_OBJECT
public:
    explicit PowerGraph(QWidget *parent = nullptr);
    void addValue(double kw);
    void setMaxKw(double maxKw);

protected:
    void paintEvent(QPaintEvent *event) override;

private:
    std::deque<double> history;
    size_t maxHistory;
    double maxPowerKw;
};

#endif