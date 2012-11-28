#include "Toggler.h"

/*------------------------------------------------------------
Method Name: Toggler
Description: Default constructor.
Parameters: N/A
Returns: Instance of Toggler
 	with states initialized to "off".
------------------------------------------------------------*/

Toggler::Toggler()
{
	m_Debouncer.Start();
	m_currentState = false; //state of the light
	m_latestInputState = false; //state of the button
}

/*------------------------------------------------------------
Method Name: Toggler
Description: Constructor that initializes currentState.
Parameters: 
	currentState: initial stored state.
Returns: Instance of Toggler with current state initialized
 to a specified value and the latest input state to "off".
------------------------------------------------------------*/
Toggler::Toggler(bool currentState)
{
	m_Debouncer.Start();
	m_currentState = currentState;
	m_latestInputState = false;
}

/*------------------------------------------------------------
Method Name: Toggler
Description: Constructor initializing both member variables.
Parameters: 
	currentState: initial stored state.
	inputState: initial latest input state.
Returns: Instance of Toggler with states initialized 
 to specified values.
------------------------------------------------------------*/
Toggler::Toggler(bool currentState, bool inputState)
{
	m_Debouncer.Start();
	m_currentState = currentState;
	m_latestInputState = inputState;
}

/*------------------------------------------------------------
Method Name: ~Toggler
Description: Destructor.
Parameters: N/A
Returns: N/A
------------------------------------------------------------*/

Toggler::~Toggler() {}

/*------------------------------------------------------------
Method Name: DoesNewStateToggle
Description: Indicates whether or not new input state 
	toggles "on", as in the moment a button is pressed, 
	and if so, toggles the stored state.
Parameters: 
	inputState: Newest state of input ex. button.
Returns: Whether or not we are seeing the input 
 change from "off" to "on".
------------------------------------------------------------*/
bool Toggler::DoesNewStateToggle(bool inputState) 
{
	bool toReturn = (inputState && (inputState != m_latestInputState));
	m_latestInputState = inputState;
	if (toReturn)
		m_currentState = !m_currentState;
	return toReturn;
}
bool Toggler::DoesNewStateToggleDBI(bool inputState) 
{
	bool toReturn = false;
	if (inputState != m_DebounceInputState)
	{ 
		m_DebounceInputState = inputState;
		m_timeLast = m_Debouncer.Get();
	}
	else if((m_timeLast + 0.33) <= m_Debouncer.Get())
	{
		toReturn = (inputState && (inputState != m_latestInputState));
		m_latestInputState = inputState;		
		if (toReturn)
			m_currentState = !m_currentState;	
	}
	return toReturn;
}
bool Toggler::DoesNewStateToggleDB(bool inputState) 
{
	bool toReturn = false;
	if((m_timeLast + 0.33) <= m_Debouncer.Get())
	{
		toReturn = (inputState && (inputState != m_latestInputState));
		m_latestInputState = inputState;		
		if (toReturn)
		{
			m_currentState = !m_currentState;
			m_timeLast = m_Debouncer.Get();
		}
	}
	return toReturn;
}

/*------------------------------------------------------------
Method Name: DoesStateChange
Description: updates stored state to input state and 
  indicates whether or not the stored state changes.
Parameters: 
	inputState: Newest state of input ex. button.
Returns: Whether or not stored state is changed.
------------------------------------------------------------*/
bool Toggler::DoesStateChange(bool inputState)
{
	bool toReturn = (inputState != m_currentState);
	m_currentState = inputState;
	m_latestInputState = inputState;
	return toReturn;
}

/*------------------------------------------------------------
Method Name: GetState
Description: Stored state accessor.
Parameters: N/A
Returns: Current stored state.
------------------------------------------------------------*/
bool Toggler::GetState()
{
	return m_currentState;
}

/*------------------------------------------------------------
Method Name: SetState
Description: Stored state mutator.
Parameters: 
	newState: new desired stored state.
Returns: N/A
------------------------------------------------------------*/
void Toggler::SetState(bool newState)
{
	m_currentState = newState;
}

/*------------------------------------------------------------
Method Name: GetLatestInputState
Description: Latest input state accessor.
Parameters: N/A
Returns: Latest input state passed in by constructors, 
 DoesStateChange, or DoesNewStateToggle.
------------------------------------------------------------*/
bool Toggler::GetLatestInputState()
{
	return m_latestInputState;
}


