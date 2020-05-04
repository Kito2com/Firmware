#include "ComState.h"
#include <iostream>
#include <cassert>
#include <px4_log.h>

ComState::ComState(INT maxStates) :
	StateMachine(maxStates)
{
}

void ComState::Cancel()
{
#if 1
	// External event functions in a state machine base class may use a partial transition
	// map if preceded by PARENT_TRANSITION. The partial transition map only includes
	// entries up to the number of states known at the current level within the
	// hierarchy. PARENT_TRANSITION must be used in anything but the most-derived
	// state machine class. The macro is as a catch-all used to capture the transition
	// to use when the state machine current state is in a parent state. The statement
	// below is read "If the Cancel event is generated when the state machine is not in
	// ST_IDLE, ST_COMPLETE, or ST_FAILED then transition to the ST_FAILED state".
	// If the base Cancel event is not appropriate for some parent classes, make Cancel()
	// virtual and define Cancel() in the derived class with a full transition map.
	PARENT_TRANSITION (ST_FAILED)

	//BEGIN_TRANSITION_MAP			              			// - Current State -
	    static const BYTE TRANSITIONS[] = {\
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_IDLE
		TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)				// ST_COMPLETED
		TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)				// ST_FAILED
	//END_TRANSITION_MAP(NULL)
	};\
	    assert(GetCurrentState() < ST_MAX_STATES); \
	ExternalEvent(TRANSITIONS[GetCurrentState()], NULL); \
	    assert((sizeof(TRANSITIONS)/sizeof(BYTE)) == ST_MAX_STATES);

#else
	// Alternatively external events can be generated manually without a
	// transition map.
	if (GetCurrentState() != ST_IDLE)
		ExternalEvent(ST_FAILED);
#endif
}

STATE_DEFINE(ComState, Idle, NoEventData)
{
	PX4_INFO("ComState::ST_Idle");
}

ENTRY_DEFINE(ComState, EntryIdle, NoEventData)
{
	PX4_INFO("ComState::EN_EntryIdle");
}

STATE_DEFINE(ComState, Completed, NoEventData)
{
	PX4_INFO("ComState::ST_Completed");
	InternalEvent(ST_IDLE);
}

STATE_DEFINE(ComState, Failed, NoEventData)
{
	PX4_INFO("ComState::ST_Failed");
	InternalEvent(ST_IDLE);
}

