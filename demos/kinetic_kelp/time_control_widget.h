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
    void setEndTime(double endTimeInSeconds);
public slots:
	void start();
	void restart();
	void playOrPause();
protected:
    void resizeEvent(QResizeEvent* e) override;
private:
    QTimer* m_timer;
	bool m_active = false;
	bool m_paused = true;
    int m_intervalMs;
    unsigned long m_ticks = 0;
	double m_time = 0;
	double m_speed = 1;
    std::optional<double> m_endTime;
    QSlider* m_scrubber = nullptr;
	QIcon m_playIcon;
	QIcon m_pauseIcon;
	QToolButton* m_playPauseButton;
	QToolButton* m_speedButton;
	void done();
signals:
    void ticked(int tick, double time);
    void restarted();
};

#endif //CARTOCROW_TIME_CONTROL_WIDGET_H
