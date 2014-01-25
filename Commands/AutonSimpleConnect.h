#ifndef AUTONSIMPLECONNECT_H
#define AUTONSIMPLECONNECT_H

#include "../CommandBase.h"

/**
 *
 *
 * @author Cheeseyx
 */
class AutonSimpleConnect: public CommandBase {
public:
	AutonSimpleConnect();
	virtual void Initialize();
	virtual void Execute();
	virtual bool IsFinished();
	virtual void End();
	virtual void Interrupted();
};

#endif
