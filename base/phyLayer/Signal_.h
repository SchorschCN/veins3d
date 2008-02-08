#ifndef SIGNAL_H_
#define SIGNAL_H_

#include <omnetpp.h>
#include "Move.h"

/**
 * TODO: Write Description at latest the modelation of the signal is final.
 */
class Signal {
protected:
	/** 
	 * This is just a temporary member to make the signal useable. 
	 * Will be adjusted as soon as the modelation of the Signal is final.
	 */
	simtime_t signalStart;	
	simtime_t signalLength;	
	Move senderMovement;
	
	
public:
	/**
	 * Returns the point in time when the receiving of the Signal started.
	 */
	simtime_t getSignalStart() const;
	
	/**
	 * Sets the point in time when the receiving of the Signal started.
	 * This methods is used by the receiving Physical layer to adjust
	 * the signal start by the propagation delay.
	 */
	void setSignalStart(simtime_t start);

	
	/**
	 * TODO: documentation
	 */
	Move getMove();
	
	/**
	 * TODO: documentation
	 */
	void setMove(Move& move);
	
	/**
	 * TODO: documentation
	 */
	simtime_t getSignalLength();
	
};

#endif /*SIGNAL_H_*/
