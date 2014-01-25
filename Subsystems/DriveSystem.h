#ifndef DRIVESYSTEM_H
#define DRIVESYSTEM_H
#include "Commands/Subsystem.h"
#include "WPILib.h"

/**
 *
 *
 * @author Cheeseyx
 */
class DriveSystem: public Subsystem {
private:
	// It's desirable that everything possible under private except
	// for methods that implement subsystem capabilities
public:
	DriveSystem();
	void InitDefaultCommand();
};

#endif
