/* 
 * File:   CStateMachine.h
 * Author: jagdeesh
 *
 * Created on September 5, 2014, 9:16 PM
 * 
 * Copy right owner: Jagdeesh Pal.
 * Contact   : jagdeesh.pal@gmail.com
 * 
 * This state machine template class is allowed to use until 
 *  above copy write notice is kept intact with this code.
 * For any bug, please report to jagdeesh.pal@gmail.com
 */

#if ! defined CSTATEMACHINE_H
#define CSTATEMACHINE_H

#include <stdint.h>
#include <map>
#include <string>
#include <list>
#include <assert.h>

#include "debug.h" //used for LOGMSG. can be removed.
#include "mutex.h" 

//check if there is any debug message defined.
#if ! define LOGMSG_DBG_ERR
    #define LOGMSG(x,y) 
#endif

/**
 * State Machine template. It is thread safe.
 * 
 * How to use it.
 * 
 * 1) declaration.
 * 
 * It needs 5 parameters:
 * 1. The class implementing state machine.
 * 2. Enum type for states
 * 3. Enum type for events
 * 4. the start state name
 * 5. a pseudo state called "StateAny" which represent any state.
 *    Some time state are switched on a event irrespective of their state.
 *    This state "StateAny" represents all states.
 * 
 * class CMyClass
 * {
 *    public :
 *      CMyClass();
 *      ~CMyClass();
 * 
 *    private:
 * 
 *     //here enum type values are defined as 4 char literal.
 *     //
 *     typedef enum { e_start='strt', e_play='play', e_pause='paus', e_stop='stop' } TEvents;
 *     typedef enum { s_start='strt', s_end='end0', s_any='any0' } TStates;
 * 
 *     CStateMachine<CMyClass,TStates,TEvents,s_start,s_any> m_my_state_machine;
 * }
 * 
 * 
 * 2) Initialization,
 *   this must be initialized before any use.
 * 
 *    e.g. m_my_state_machine.intialize();
 */

template< class StateMachineClass
    , typename TStateName
    , typename TEventName
    , TStateName StartState
    , TStateName StateAny> class CStateMachine : private mutex
{    
private:
    typedef bool (StateMachineClass::*StateHandlers)(TStateName state);    
    typedef struct  _TStateActionHandlers
    {
        StateHandlers m_entry_action_handler;
        StateHandlers m_exit_action_handler;
        StateHandlers m_do_action_handler;  
        TStateName                  m_state;
        _TStateActionHandlers()
        {
            m_state = StartState;
            m_entry_action_handler = 0;
            m_exit_action_handler = 0;
            m_do_action_handler = 0;
        }
    }TStateActionHandlers;

    typedef std::map<TStateName,TStateActionHandlers*>TStateHandlers;    
    typedef std::map<uint64_t,TStateActionHandlers*>TTransitionMap;

    StateMachineClass          *m_context;                    
    TStateHandlers             m_state_handler;    
    TTransitionMap             m_state_transion_map;   
    std::list<TEventName>      m_event_queue;    
    bool                       m_handling_event;
    TStateActionHandlers *m_current_state;

    int                        m_event_count;
public:        
    CStateMachine(StateMachineClass *context)
    :m_context(context)
    ,m_current_state(0)
    ,m_event_count(0)
    {       
    }
    
    virtual ~CStateMachine(){}

    void initialize()
    {
        m_state_handler[StartState] = m_current_state = new TStateActionHandlers();             
        m_current_state->m_state = StartState;
    }

//------------------------------------------------------------------------------

/**
 * Register state handler routines.
 * 
 * @param state the name of state
 * @param entry entry state handler. Can be null.
 * @param exit exit state handler. Can be null.
 * @param action action handler of state. This must not be null.
 */    
void add_state_handler(TStateName state
                       , StateHandlers entry
                       , StateHandlers exit
                       , StateHandlers action)
{
    TStateActionHandlers *state_handler = new TStateActionHandlers();
    state_handler->m_state = state;
    state_handler->m_do_action_handler = action;
    state_handler->m_entry_action_handler = entry;
    state_handler->m_exit_action_handler  = exit;
        
    m_state_handler[ state ] = state_handler;
    
    assert(action);
}

//------------------------------------------------------------------------------

/**
 * Register state transition. an entry will be in state transition table.
 * 
 * @param state  the name of state.
 * @param event  the name of event for transition.
 * @param next_state the name of next state transited on given event.
 * @return true. on success. assert(0) on duplicate.
 * 
 * Note: a special state StateAny represents all states.
 */
bool add_state_transition(TStateName state,
                          TEventName event,
                          TStateName next_state)
{
    bool ok = false;
    
    typename TStateHandlers::iterator its  = m_state_handler.find( state );
    typename TStateHandlers::iterator itn  = m_state_handler.find( next_state );
    if ( (its !=  m_state_handler.end() || state == StateAny)
        && ( itn  != m_state_handler.end() ))
    {
        uint64_t map_id = (uint64_t)state | ((uint64_t)event << sizeof(TStateName));

        m_state_transion_map[ map_id ] = itn->second;
        
        ok = true;
    }
    else
    {
        assert(0);
    }
    return ok;
}

//------------------------------------------------------------------------------

/**
 * Register an event handler. The action handler of the state will be called on 
 * the event.
 * 
 * @param state the name of state
 * @param event the name of event to be handled in the given state
 * @return true  on success. assert(0) on duplicate event registration.
 */
bool add_state_event_action(TStateName state,
                          TEventName event)
{
    bool ok = false;    
    typename TStateHandlers::iterator its  = m_state_handler.find( state );
    if ( its !=  m_state_handler.end() )
    {
        uint64_t map_id = (uint64_t)state | ((uint64_t)event << sizeof(TStateName));

        m_state_transion_map[ map_id ] = 0; //in this case only state action is used 
        
        ok = true;
    }
    else
    {
        assert(0);
    }
    
    return ok;
}

//------------------------------------------------------------------------------

bool queue_event(const TEventName event)
{
    lock();

    m_event_count++;
    m_event_queue.push_back(event);

    unlock();
    
    return true;
}

//------------------------------------------------------------------------------

bool handle_event()
{
    if ( m_event_count )
    {
        lock();

        TEventName id = *m_event_queue.begin();
        m_event_count--;
        m_event_queue.pop_front();

        unlock();
        
        handle_event(id);    
        
        return true;
    }    
    return false;
}

//------------------------------------------------------------------------------

inline bool execute_current_state_action()
{
    return (m_context->*(m_current_state->m_do_action_handler))( m_current_state->m_state );
}

//------------------------------------------------------------------------------

bool handle_event(const TEventName event)
{
    if ( m_handling_event )
    {
        return queue_event(event);
    }
    
    m_handling_event = true;
    
    bool ok = false;
	uint64_t map_id = (uint64_t)m_current_state->m_state | ((uint64_t)event<<sizeof(TStateName));

    typename TTransitionMap::iterator itn = m_state_transion_map.find(map_id);

    if ( itn ==  m_state_transion_map.end() ) 
    {
        //from all states to this new state
        //   
        map_id = (uint64_t)StateAny | ((uint64_t)event<<sizeof(TStateName));    
        itn = m_state_transion_map.find(map_id);
    }

    const char *sst=(char *)&m_current_state->m_state;
    const char *evt=(char *)&event;                
    
    if ( itn !=  m_state_transion_map.end() )
    {           
        //no transition specified
        //current action hanlder is treated as event handler
        if( 0 == itn->second )
        {
            LOGMSG(DBG_INFO,("STATE MACHINE: Current State '%c%c%c%c', event '%c%c%c%c'"
                        ,sst[3],sst[2],sst[1],sst[0]
                        ,evt[3],evt[2],evt[1],evt[0]));
            
            bool ok = execute_current_state_action();
            
            m_handling_event = false;
            
            return ok ;
        }

        const char *est=(char *)&itn->second->m_state;
        const char *evt=(char *)&event;
        
        //print transition info.
        LOGMSG(DBG_INFO,("STATE MACHINE: Current State '%c%c%c%c', event '%c%c%c%c', next state '%c%c%c%c'"
                        ,sst[3],sst[2],sst[1],sst[0]
                        ,evt[3],evt[2],evt[1],evt[0]
                        ,est[3],est[2],est[1],est[0]));
                
        //exit action
        if ( m_current_state->m_exit_action_handler )
        {
            (m_context->*(m_current_state->m_exit_action_handler))(m_current_state->m_state);
        }
        
        //state change
        m_current_state = itn->second;
        
        //entry action
        if ( m_current_state->m_entry_action_handler )
        {
            ok = (m_context->*(m_current_state->m_entry_action_handler))(m_current_state->m_state);
            
            //entry can cancel to the do action if it failed.
        }
        
        //do action.
        if ( ok && m_current_state->m_do_action_handler )
        {
            (m_context->*(m_current_state->m_do_action_handler))(m_current_state->m_state);
        }
        
        ok = true;
    }
    else
    {
            LOGMSG(DBG_INFO,("STATE MACHINE: No Transition defined for state '%c%c%c%c', event '%c%c%c%c'"
                        ,sst[3],sst[2],sst[1],sst[0]
                        ,evt[3],evt[2],evt[1],evt[0]));
    }
    
    m_handling_event = false;
    
    return ok; 
}
};

#endif /* CSTATEMACHINE_H */
