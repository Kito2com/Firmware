#include "EstState.h"
#include <iostream>
#include <cassert>
#include <px4_log.h>

EstState::EstState() :
	ComState(ST_MAX_STATES),
	m_pollActive(FALSE),
	m_count(0)
{
}

void EstState::Start()
{
	//BEGIN_TRANSITION_MAP			              			// - Current State -
	    static const BYTE TRANSITIONS[] = {\
		TRANSITION_MAP_ENTRY (ST_RAW_DATA_STATISTIC_INITIAL)		// ST_IDLE
		TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)				// ST_COMPLETED
		TRANSITION_MAP_ENTRY (CANNOT_HAPPEN)				// ST_FAILED
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_RAW_DATA_STATISTIC_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_RAW_DATA_STATISTIC
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_ALIGNMENT
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_PRE_STATIC_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_PRE_STATIC_FILTER
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_STATIC_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_STATIC_FILTER
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_PRE_TAKEOFF_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_PRE_TAKEOFF_FILTER
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_IN_AIR_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_IN_AIR_FILTER
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_POST_LAND_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_POST_LAND_FILTER
	//END_TRANSITION_MAP(NULL)
	    };\
		assert(GetCurrentState() < ST_MAX_STATES); \
	    ExternalEvent(TRANSITIONS[GetCurrentState()], NULL); \
		assert((sizeof(TRANSITIONS)/sizeof(BYTE)) == ST_MAX_STATES);
}

void EstState::Poll()
{
	//BEGIN_TRANSITION_MAP			              			// - Current State -
	    static const BYTE TRANSITIONS[] = {\
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_IDLE
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_COMPLETED
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_FAILED
		TRANSITION_MAP_ENTRY (ST_RAW_DATA_STATISTIC)			// ST_RAW_DATA_STATISTIC_INITIAL
		TRANSITION_MAP_ENTRY (ST_RAW_DATA_STATISTIC)			// ST_RAW_DATA_STATISTIC
		TRANSITION_MAP_ENTRY (EVENT_IGNORED)				// ST_ALIGNMENT
		TRANSITION_MAP_ENTRY (ST_PRE_STATIC_FILTER)			// ST_PRE_STATIC_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (ST_PRE_STATIC_FILTER)			// ST_PRE_STATIC_FILTER
		TRANSITION_MAP_ENTRY (ST_STATIC_FILTER)				// ST_STATIC_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (ST_STATIC_FILTER)				// ST_STATIC_FILTER
		TRANSITION_MAP_ENTRY (ST_PRE_TAKEOFF_FILTER)			// ST_PRE_TAKEOFF_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (ST_PRE_TAKEOFF_FILTER)			// ST_PRE_TAKEOFF_FILTER
		TRANSITION_MAP_ENTRY (ST_IN_AIR_FILTER)				// ST_IN_AIR_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (ST_IN_AIR_FILTER)				// ST_IN_AIR_FILTER
		TRANSITION_MAP_ENTRY (ST_POST_LAND_FILTER)			// ST_POST_LAND_FILTER_INITIAL
		TRANSITION_MAP_ENTRY (ST_POST_LAND_FILTER)			// ST_POST_LAND_FILTER
	//END_TRANSITION_MAP(NULL)
	    };\
	    assert(GetCurrentState() < ST_MAX_STATES); \
	  ExternalEvent(TRANSITIONS[GetCurrentState()], NULL); \
	  assert((sizeof(TRANSITIONS)/sizeof(BYTE)) == ST_MAX_STATES);
}

// Start the estimating state.
STATE_DEFINE(EstState, RawDataStatisticsInitial, NoEventData)
{
	PX4_INFO("EstState::ST_RawDataStatisticsInitial");

	// Start polling while RawDataStatistics
	StartPoll();
}

// Guard condition to detemine whether RawDataStatisticsInitial state is executed.
GUARD_DEFINE(EstState, GuardRawDataStatisticsInitial, NoEventData)
{
	PX4_INFO("EstState::GD_GuardRawDataStatisticsInitial");
	if (m_armState == ArmState::standby)
		return TRUE;	// OK to start estimating.
	else
		return FALSE;	// Can't start estimating.
}

// While RawDataStatistics continues.
STATE_DEFINE(EstState, RawDataStatistics, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_RawDataStatistics : Count is %i", m_count+1);
	    PX4_INFO("Gyro :\t%g\t%g\t%g", (double)gyro_cur[0],  (double)gyro_cur[1],  (double)gyro_cur[2]);
	    PX4_INFO("Gyro_dt :\t%u", gyro_dt_cur);
	    PX4_INFO("Acc :\t%g\t%g\t%g",  (double)acc_cur[0],  (double)acc_cur[1],  (double)acc_cur[2]);
	    PX4_INFO("Acc_dt :\t%u", acc_dt_cur);
	  }

	int dt = (gyro_dt_cur / 100) * 100;
	hist[dt]++;

	m_count ++;
	if (m_count >= 1000)
	  {
	    PX4_INFO("EstState::ST_RawDataStatistics : Count is %i", m_count);
	    PX4_INFO("Gyro :\t%g\t%g\t%g", (double)gyro_cur[0],  (double)gyro_cur[1],  (double)gyro_cur[2]);
	    PX4_INFO("Gyro_dt :\t%u", gyro_dt_cur);
	    PX4_INFO("Acc :\t%g\t%g\t%g",  (double)acc_cur[0],  (double)acc_cur[1],  (double)acc_cur[2]);
	    PX4_INFO("Acc_dt :\t%u", acc_dt_cur);

	    for (auto h : hist)
	      {
		PX4_INFO("%d\t:\t%d", h.first, h.second);
	      }

	    InternalEvent(ST_ALIGNMENT);
	  }
}

// Exit action when RawDataStatistics state exits.
EXIT_DEFINE(EstState, ExitRawDataStatistics)
{
	PX4_INFO("EstState::EX_ExitRawDataStatistics");

	// RawDataStatistics over, stop polling
	m_count = 0;
	StopPoll();
}

// Enter Alignment state.
STATE_DEFINE(EstState, Alignment, NoEventData)
{
	PX4_INFO("EstState::ST_Alignment");

	InternalEvent(ST_PRE_STATIC_FILTER_INITIAL);
}

// Enter PreStaticFilterInitial state.
STATE_DEFINE(EstState, PreStaticFilterInitial, NoEventData)
{
	PX4_INFO("EstState::ST_PreStaticFilterInitial");

	// Start polling while PreStaticFilter
	StartPoll();
}

// While PreStaticFilter continues.
STATE_DEFINE(EstState, PreStaticFilter, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_PreStaticFilter : Count is %i", m_count+1);
	  }
	m_count ++;
	if (m_count >= 100)
	  {
	    PX4_INFO("EstState::ST_PreStaticFilter : Count is %i", m_count);
	    InternalEvent(ST_STATIC_FILTER_INITIAL);
	  }
}

// Exit action when PreStaticFilter state exits.
EXIT_DEFINE(EstState, ExitPreStaticFilter)
{
	PX4_INFO("EstState::EX_ExitPreStaticFilter");

	// PreStaticFilter over, stop polling
	m_count = 0;
	StopPoll();
}

// Enter StaticFilterInitial state.
STATE_DEFINE(EstState, StaticFilterInitial, NoEventData)
{
	PX4_INFO("EstState::ST_StaticFilterInitial");

	// Start polling while PreStaticFilter
	StartPoll();
}

// While StaticFilter continues.
STATE_DEFINE(EstState, StaticFilter, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_StaticFilter : Count is %i", m_count+1);
	  }
	m_count ++;

	if (m_armState == ArmState::armed)
	  {
	    PX4_INFO("EstState::ST_StaticFilter : Count is %i", m_count);
	    InternalEvent(ST_PRE_TAKEOFF_FILTER_INITIAL);
	  }
}

// Exit action when StaticFilter state exits.
EXIT_DEFINE(EstState, ExitStaticFilter)
{
	PX4_INFO("EstState::EX_ExitStaticFilter");

	// StaticFilter over, stop polling
	m_count = 0;
	StopPoll();
}

// Enter PreTakeoffFilterInitial state.
STATE_DEFINE(EstState, PreTakeoffFilterInitial, NoEventData)
{
	PX4_INFO("EstState::ST_PreTakeoffFilterInitial");

	// Start polling while PreTakeoffFilter
	StartPoll();
}

// While PreTakeoffFilter continues.
STATE_DEFINE(EstState, PreTakeoffFilter, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_PreTakeoffFilter : Count is %i", m_count+1);
	  }
	m_count ++;
	if (m_count >= 20)
	  {
	    PX4_INFO("EstState::ST_PreTakeoffFilter : Count is %i", m_count);
	    InternalEvent(ST_IN_AIR_FILTER_INITIAL);
	  }
}

// Exit action when PreTakeoffFilter state exits.
EXIT_DEFINE(EstState, ExitPreTakeoffFilter)
{
	PX4_INFO("EstState::EX_ExitPreTakeoffFilter");

	// PreTakeoffFilter over, stop polling
	m_count = 0;
	StopPoll();
}

// Enter InAirFilterInitial state.
STATE_DEFINE(EstState, InAirFilterInitial, NoEventData)
{
	PX4_INFO("EstState::InAirFilterInitial");

	// Start polling while InAirFilter
	StartPoll();
}

// While InAirFilter continues.
STATE_DEFINE(EstState, InAirFilter, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_InAirFilter : Count is %i", m_count+1);
	  }
	m_count ++;
	if (++m_count >= 1000)
	  {
	    PX4_INFO("EstState::ST_InAirFilter : Count is %i", m_count);
	    InternalEvent(ST_POST_LAND_FILTER_INITIAL);
	  }
}

// Exit action when InAirFilter state exits.
EXIT_DEFINE(EstState, ExitInAirFilter)
{
	PX4_INFO("EstState::EX_ExitInAirFilter");

	// InAirFilter over, stop polling
	m_count = 0;
	StopPoll();
}


// Enter PostLandFilterInitial state.
STATE_DEFINE(EstState, PostLandFilterInitial, NoEventData)
{
	PX4_INFO("EstState::ST_PostLandFilterInitial");

	// Start polling while PostLandFilter
	StartPoll();
}

// While PostLandFilter continues.
STATE_DEFINE(EstState, PostLandFilter, NoEventData)
{
	if (m_count == 0)
	  {
	    PX4_INFO("EstState::ST_PostLandFilter : Count is %i", m_count+1);
	  }
	m_count ++;
	if (m_armState == ArmState::standby)
	  {
	    PX4_INFO("EstState::ST_PostLandFilter : Count is %i", m_count);
	    InternalEvent(ST_COMPLETED);
	  }
}

// Exit action when PostLandFilter state exits.
EXIT_DEFINE(EstState, ExitPostLandFilter)
{
	PX4_INFO("EstState::EX_ExitPostLandFilter");

	// PostLandFilter over, stop polling
	StopPoll();
}
