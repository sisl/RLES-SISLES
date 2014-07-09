# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/09/2014


module AbstractEncounterModelImpl

export AbstractEncounterModel

abstract AbstractEncounterModel

end


module AbstractEncounterModelInterfaces

export generateEncountersToFile,
       generateEncounter,
       getInitialState,
       getNextCommand,
       getTrajectory

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

end


