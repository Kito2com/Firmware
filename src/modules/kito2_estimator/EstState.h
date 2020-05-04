#ifndef _EST_STATE_H
#define _EST_STATE_H

#include "ComState.h"
#include <cassert>

#include <map>

enum class ArmState {init, standby, armed};

// @brief EstState shows StateMachine features including state machine
// inheritance, state function override, and guard/entry/exit actions. ComState
// provides common states shared with EstState.
class EstState : public ComState
{
public:
	EstState();

	virtual void Start();
	void Poll();

	BOOL IsPollActive() { return m_pollActive; }

	void SetArmState(ArmState armState) { m_armState = armState; }

	void SetGyroData(float gyro[3], float dt) {
	  gyro_cur[0] = gyro[0];
	  gyro_cur[1] = gyro[1];
	  gyro_cur[2] = gyro[2];
	  gyro_dt_cur = dt;
	}

	void SetAccData(float acc[3], float dt) {
	  acc_cur[0] = acc[0];
	  acc_cur[1] = acc[1];
	  acc_cur[2] = acc[2];
	  acc_dt_cur = dt;
	}

private:
	BOOL m_pollActive;
	INT m_count;
	ArmState m_armState {ArmState::init} ;

	float gyro_cur[3] {0.0, 0.0, 0.0};
	float gyro_ave[3] {0.0, 0.0, 0.0};
	float gyro_dev[3] {0.0, 0.0, 0.0};
	float gyro_bias[3] {0.0, 0.0, 0.0};
	UINT32 gyro_dt_cur {0};
	float gyro_dt_ave {0.0};
	float gyro_dt_dev {0.0};

	float acc_cur[3] {0.0, 0.0, 0.0};
	float acc_ave[3] {0.0, 0.0, 0.0};
	float acc_dev[3] {0.0, 0.0, 0.0};
	float acc_bias[3] {0.0, 0.0, 0.0};
	UINT32 acc_dt_cur {0};
	float acc_dt_ave {0.0};
	float acc_dt_dev {0.0};

	std::map<int, int> hist;

	void StartPoll() { m_pollActive = TRUE; }
	void StopPoll() { m_pollActive = FALSE; }

	// State enumeration order must match the order of state method entries
	// in the state map.
	enum States
	{
		// Continue state numbering using the last ComState::States enum value
		ST_RAW_DATA_STATISTIC_INITIAL = ComState::ST_MAX_STATES,
		ST_RAW_DATA_STATISTIC,
		ST_ALIGNMENT,
		ST_PRE_STATIC_FILTER_INITIAL,
		ST_PRE_STATIC_FILTER,
		ST_STATIC_FILTER_INITIAL,
		ST_STATIC_FILTER,
		ST_PRE_TAKEOFF_FILTER_INITIAL,
		ST_PRE_TAKEOFF_FILTER,
		ST_IN_AIR_FILTER_INITIAL,
		ST_IN_AIR_FILTER,
		ST_POST_LAND_FILTER_INITIAL,
		ST_POST_LAND_FILTER,
		ST_MAX_STATES
	};

	// Define the state machine state functions with event data type
	STATE_DECLARE(EstState, 	RawDataStatisticsInitial,	NoEventData)
	GUARD_DECLARE(EstState, 	GuardRawDataStatisticsInitial,	NoEventData)  // Guard
	STATE_DECLARE(EstState, 	RawDataStatistics,		NoEventData)
	EXIT_DECLARE(EstState,		ExitRawDataStatistics)	// Exit
	STATE_DECLARE(EstState, 	Alignment,			NoEventData)
	STATE_DECLARE(EstState, 	PreStaticFilterInitial,		NoEventData)
	STATE_DECLARE(EstState, 	PreStaticFilter,		NoEventData)
	EXIT_DECLARE(EstState,		ExitPreStaticFilter)	// Exit
	STATE_DECLARE(EstState, 	StaticFilterInitial,		NoEventData)
	STATE_DECLARE(EstState, 	StaticFilter,			NoEventData)
	EXIT_DECLARE(EstState,		ExitStaticFilter)	// Exit
	STATE_DECLARE(EstState, 	PreTakeoffFilterInitial,	NoEventData)
	STATE_DECLARE(EstState, 	PreTakeoffFilter,		NoEventData)
	EXIT_DECLARE(EstState,		ExitPreTakeoffFilter)	// Exit
	STATE_DECLARE(EstState, 	InAirFilterInitial,		NoEventData)
	STATE_DECLARE(EstState, 	InAirFilter,			NoEventData)
	EXIT_DECLARE(EstState,		ExitInAirFilter)	// Exit
	STATE_DECLARE(EstState, 	PostLandFilterInitial,		NoEventData)
	STATE_DECLARE(EstState, 	PostLandFilter,			NoEventData)
	EXIT_DECLARE(EstState,		ExitPostLandFilter)	// Exit

	// State map to define state object order. Each state map entry defines a
	// state object.
	//BEGIN_STATE_MAP_EX
	private:\
	virtual const StateMapRow* GetStateMap() { return NULL; }\
	virtual const StateMapRowEx* GetStateMapEx() {\
		static const StateMapRowEx STATE_MAP[] = {

		STATE_MAP_ENTRY_ALL_EX(&Idle, 0, &EntryIdle, 0)
		STATE_MAP_ENTRY_EX(&Completed)
		STATE_MAP_ENTRY_EX(&Failed)
		STATE_MAP_ENTRY_ALL_EX(&RawDataStatisticsInitial, &GuardRawDataStatisticsInitial, 0, 0)
		STATE_MAP_ENTRY_ALL_EX(&RawDataStatistics, 0, 0, &ExitRawDataStatistics)
		STATE_MAP_ENTRY_EX(&Alignment)
		STATE_MAP_ENTRY_EX(&PreStaticFilterInitial)
		STATE_MAP_ENTRY_ALL_EX(&PreStaticFilter, 0, 0, &ExitPreStaticFilter)
		STATE_MAP_ENTRY_EX(&StaticFilterInitial)
		STATE_MAP_ENTRY_ALL_EX(&StaticFilter, 0, 0, &ExitStaticFilter)
		STATE_MAP_ENTRY_EX(&PreTakeoffFilterInitial)
		STATE_MAP_ENTRY_ALL_EX(&PreTakeoffFilter, 0, 0, &ExitPreTakeoffFilter)
		STATE_MAP_ENTRY_EX(&InAirFilterInitial)
		STATE_MAP_ENTRY_ALL_EX(&InAirFilter, 0, 0, &ExitInAirFilter)
		STATE_MAP_ENTRY_EX(&PostLandFilterInitial)
		STATE_MAP_ENTRY_ALL_EX(&PostLandFilter, 0, 0, &ExitPostLandFilter)
	//END_STATE_MAP_EX
		}; \
		    assert((sizeof(STATE_MAP)/sizeof(StateMapRowEx)) == ST_MAX_STATES); \
	       return &STATE_MAP[0]; }

};

#endif
