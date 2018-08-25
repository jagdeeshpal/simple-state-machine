# simple-state-machine
C++ Finit State Machine handling template.

/** How to use it.
 *
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
 
