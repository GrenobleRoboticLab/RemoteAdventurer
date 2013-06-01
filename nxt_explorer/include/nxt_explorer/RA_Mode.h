#ifndef RA_MODE_H__
#define RA_MODE_H__

#include <nxt_adventurer/RA_Dashboard.h>
#include <set>

using namespace RemoteAdventurer;

namespace RemoteAdventurerExplorer {

class Mode : public DashboardListener {
private:
    enum ModeState {
        MS_STARTED,
        MS_STOPED
    };

public:
    Mode();
    ~Mode();

    void        start() { m_wState = MS_STARTED;    }
    void        stop()  { m_wState = MS_STOPED;     }

    ModeState   getState() const { return m_wState; }

protected:
    void        release() = 0;

private:
    ModeState   m_wState;
};

} // namespace RemoteAdventurerExplorer

#endif // RA_MODE_H__
