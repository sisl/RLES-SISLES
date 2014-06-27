# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/19/2014


module Interfaces

export
    # Common API
    initialize,
    step,

    # Encounter
    generateEncountersToFile,
    generateEncounter,
    getInitialState,
    getNextCommand,
    getTrajectory,

    # Pilot Response
    updatePilotResponse,

    # Dynamic Model
    initializeDynamicModel,
    simulateDynamicModel,

    # World Model
    updateObjectState,
    getAllObjectStates,
    getAll,
    updateAll,

    # Sensor
    updateSensor,

    # Collision Avoidance System
    testThreat,
    selectRA

# Common API

function initialize()
end

function step()
end


# Encounter

function generateEncountersToFile()
end

function generateEncounter()
end

function getInitialState()
end

function getNextCommand()
end

function getTrajectory()
end


# Dynamic Model

function initializeDynamicModel()
end

function simulateDynamicModel()
end


# Pilot Response
function updatePilotResponse()
end


# World Model
function updateObjectState()
end

function getAllObjectStates()
end

function getAll()
end

function updateAll()
end


# Sensor
function updateSensor()
end


# Collision Avoidance System
function testThreat()
end

function selectRA()
end

end


