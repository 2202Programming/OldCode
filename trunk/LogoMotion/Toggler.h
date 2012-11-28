#ifndef TOGGLER_H_
#define TOGGLER_H_
#include "Timer.h"
class Toggler
{
public:
	Toggler();
	Toggler(bool currentState);
	Toggler(bool currentState, bool inputState);
	virtual ~Toggler();
	bool DoesNewStateToggle(bool inputState);
	bool DoesNewStateToggleDB(bool inputState);
	bool DoesNewStateToggleDBI(bool inputState);
	bool DoesStateChange(bool inputState);
	bool GetState();
	void SetState(bool newState);
	bool GetLatestInputState();
private:
	bool m_currentState;
	bool m_latestInputState;
	double m_timeLast;
	Timer m_Debouncer;
	bool m_DebounceInputState;
	
};

#endif

