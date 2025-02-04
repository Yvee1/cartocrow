#include "time_control_widget.h"

void TimeControlToolBar::tick() {
    ++m_ticks;
    if (m_scrubber != nullptr && m_endTime.has_value()) {
        m_scrubber->setValue(m_ticks * m_intervalMs / *m_endTime);
    }
    parentWidget()->update();
}

double TimeControlToolBar::time() const {
    return m_ticks * m_intervalMs / 1000.0;
}

TimeControlToolBar::TimeControlToolBar(QWidget* parent, std::optional<double> endTimeSecond, int intervalMs) :
QToolBar(parent), m_intervalMs(intervalMs), m_endTime(endTimeSecond) {
    auto* restartButton = new QToolButton(this);
    restartButton->setText("↺");
    addWidget(restartButton);
    auto* playPauseButton = new QToolButton(this);
    playPauseButton->setText("||");
    addWidget(playPauseButton);

    restartButton->setMaximumSize(100, 50);
    restartButton->setContentsMargins(0, 0, 0, 0);
    playPauseButton->setMaximumSize(100, 50);
    playPauseButton->setContentsMargins(0, 0, 0, 0);

    if (m_endTime.has_value()) {
        m_scrubber = new QSlider();
        m_scrubber->setOrientation(Qt::Horizontal);
        addWidget(m_scrubber);
        m_scrubber->setEnabled(false);
        m_scrubber->setMinimum(0);
        m_scrubber->setMaximum(1000);
    }

    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, [this] { tick(); });
    m_timer->start(intervalMs);

    connect(playPauseButton, &QToolButton::clicked, [this, playPauseButton](){
        if (!m_pausedTime.has_value()) {
            m_timer->stop();
            m_pausedTime = m_ticks * m_intervalMs;
            playPauseButton->setText("▶");
        } else {
            m_timer->start();
            m_pausedTime = std::nullopt;
            playPauseButton->setText("||");
        }
    });
    connect(restartButton, &QToolButton::clicked, [this](){
        m_pausedTime = std::nullopt;
        m_ticks = 0;
        m_timer->start();
    });
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