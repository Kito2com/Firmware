#ifndef _COM_STATE_H
#define _COM_STATE_H

#include "StateMachine.h"

/// @brief ComState is a subclass state machine for other self tests to
/// inherit from. The class has common states for all derived classes to
/// share.
class ComState : public StateMachine
{
public:
	ComState(INT maxStates);

	virtual void Start() = 0;
	void Cancel();

protected:
	// State enumeration order must match the order of state method entries
	// in the state map.
	enum States
	{
		ST_IDLE,
		ST_COMPLETED,
		ST_FAILED,
		ST_MAX_STATES
	};

	// Define the state machine states
	STATE_DECLARE(ComState, 	Idle,			NoEventData)
	ENTRY_DECLARE(ComState, 	EntryIdle,		NoEventData)
	STATE_DECLARE(ComState, 	Completed,		NoEventData)
	STATE_DECLARE(ComState, 	Failed,			NoEventData)


};

#endif
