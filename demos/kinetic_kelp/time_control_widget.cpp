#include "time_control_widget.h"
#include <QApplication>
#include <QStyle>
#include <iostream>

void TimeControlToolBar::tick() {
	++m_ticks;
	m_time += m_speed * m_intervalMs / 1000.0;
    if (m_scrubber != nullptr && m_endTime.has_value()) {
        int timeScrubber = (m_time - m_startTime) / *m_endTime * 1000.0;
        m_scrubber->setValue(timeScrubber);
    }
	emit ticked(m_ticks, m_time);
    parentWidget()->update();

	if (m_endTime.has_value() && m_time >= *m_endTime) {
		done();
	}
}

double TimeControlToolBar::time() const {
    return m_time;
}

TimeControlToolBar::TimeControlToolBar(QWidget* parent, double startTimeSecond, std::optional<double> endTimeSecond, int intervalMs) :
QToolBar(parent), m_intervalMs(intervalMs), m_startTime(startTimeSecond), m_endTime(endTimeSecond) {
	m_speedButton = new QToolButton(this);
	m_speedButton->setText("1x");
	addWidget(m_speedButton);

	m_playIcon = QApplication::style()->standardIcon(QStyle::SP_MediaPlay);
	m_pauseIcon = QApplication::style()->standardIcon(QStyle::SP_MediaPause);
	QIcon restartIcon = QApplication::style()->standardIcon(QStyle::SP_MediaSkipBackward);
    m_restartButton = new QToolButton(this);
	m_restartButton->setIcon(restartIcon);
    addWidget(m_restartButton);
    m_playPauseButton = new QToolButton(this);
	m_playPauseButton->setIcon(m_playIcon);

    m_restartButton->setMaximumSize(100, 50);
    m_restartButton->setContentsMargins(0, 0, 0, 0);
    m_playPauseButton->setMaximumSize(100, 50);
    m_playPauseButton->setContentsMargins(0, 0, 0, 0);

    if (m_endTime.has_value()) {
        m_scrubber = new QSlider();
        m_scrubber->setOrientation(Qt::Horizontal);
        addWidget(m_scrubber);
        m_scrubber->setEnabled(false);
        m_scrubber->setMinimum(0);
        m_scrubber->setMaximum(1000);
    }
	addWidget(m_playPauseButton);

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, [this] { tick(); });
    connect(m_playPauseButton, &QToolButton::clicked, this, &TimeControlToolBar::playOrPause);
    connect(m_restartButton, &QToolButton::clicked, this, &TimeControlToolBar::restart);
	connect(m_speedButton, &QToolButton::clicked, this, [this]() {
		std::vector<double> speeds({0.25, 0.5, 1.0, 1.5, 2.0, 100.0});
		auto it = std::lower_bound(speeds.begin(), speeds.end(), m_speed);
		++it;
		if (it == speeds.end()) {
			it = speeds.begin();
		}
		m_speed = *it;
		std::stringstream ss;
		ss << std::setprecision(2) << m_speed << "x";
		m_speedButton->setText(ss.str().c_str());
	});
}

void TimeControlToolBar::restart() {
	m_ticks = 0;
	m_time = m_startTime;
	m_timer->stop();
	m_active = false;
	if (m_scrubber != nullptr) {
		m_scrubber->setValue(0);
	}
	m_playPauseButton->setIcon(m_playIcon);
    emit restarted();
	parentWidget()->update();
}

void TimeControlToolBar::start() {
	m_ticks = 0;
	m_time = m_startTime;
	m_timer->start(m_intervalMs);
}

void TimeControlToolBar::playOrPause() {
	if (m_active) {
		m_timer->stop();
		m_playPauseButton->setIcon(m_playIcon);
		m_paused = true;
	} else {
		if (!m_paused) {
			m_ticks = 0;
			m_time = m_startTime;
			emit restarted();
		}
		m_timer->start(m_intervalMs);
		m_playPauseButton->setIcon(m_pauseIcon);
		m_paused = false;
		parentWidget()->update();
	}
	m_active = !m_active;
}

void TimeControlToolBar::done() {
	m_timer->stop();
	m_playPauseButton->setIcon(m_playIcon);
	m_active = false;
}

void TimeControlToolBar::resized() {
    QRect r = parentWidget()->rect();
    QSize toolBarSize = sizeHint();
    QRect toolBarRect(QPoint{r.right() - toolBarSize.width(), r.top()},
                      QPoint{r.right(), r.top() + toolBarSize.height()});
    setGeometry(toolBarRect);
}

void TimeControlToolBar::resizeEvent(QResizeEvent* e) {
    resized();
}

void TimeControlToolBar::setStartTime(double startTimeInSeconds) {
    m_startTime = startTimeInSeconds;
}

void TimeControlToolBar::setEndTime(double endTimeInSeconds) {
    m_endTime = endTimeInSeconds;
}

void TimeControlToolBar::setPlayPauseEnabled(bool enabled) {
	m_playPauseButton->setEnabled(enabled);
}

void TimeControlToolBar::setRestartEnabled(bool enabled) {
	m_restartButton->setEnabled(enabled);
}

void TimeControlToolBar::setInterval(int intervalMs) {
	m_intervalMs = intervalMs;
	if (m_active) {
		m_timer->start(m_intervalMs);
	}
}
