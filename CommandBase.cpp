#include "CommandBase.h"
#include "Subsystems/ExampleSubsystem.h"
#include "Subsystems/DriveSystem.h"
#include "Commands/Scheduler.h"

CommandBase::CommandBase(const char *name) : Command(name) {
}

CommandBase::CommandBase() : Command() {
}

// Initialize a single static instance of all of your subsystems to NULL
ExampleSubsystem* CommandBase::examplesubsystem = NULL;
DriveSystem* CommandBase::drivesys = NULL;

OI* CommandBase::oi = NULL;

void CommandBase::init() {
    // Create a single static instance of all of your subsystems. The following
	// line should be repeated for each subsystem in the project.
	examplesubsystem = new ExampleSubsystem();
	
	drivesys = new DriveSystem();
	
	
	oi = new OI();
}
