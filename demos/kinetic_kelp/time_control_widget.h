#ifndef CARTOCROW_TIME_CONTROL_WIDGET_H
#define CARTOCROW_TIME_CONTROL_WIDGET_H

#include <QToolBar>
#include <QToolButton>
#include <QTimer>
#include <QWidget>
#include <QSlider>

class TimeControlToolBar : public QToolBar {
Q_OBJECT;

public:
    void tick();
    double time() const;

    TimeControlToolBar(QWidget* parent = nullptr, std::optional<double> endTimeSecond = std::nullopt, int intervalMs = 17);

    void resized();
protected:
    void resizeEvent(QResizeEvent* e) override;
private:
    QTimer* m_timer;
    std::optional<double> m_pausedTime;
    int m_intervalMs;
    unsigned long m_ticks = 0;
    std::optional<double> m_endTime;
    QSlider* m_scrubber = nullptr;
};

#endif //CARTOCROW_TIME_CONTROL_WIDGET_H
