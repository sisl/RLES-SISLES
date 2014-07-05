# Author: Youngjun Kim, youngjun@stanford.edu
# Date: 06/25/2014

using SISLES
using HDF5, JLD


bTCAS = false
bCompareTCAS = false
bValidate = false

bGenerateSamplesToFile = false
bReadSampleFromFile = false
sample_number = 1


SISLES_PATH = Pkg.dir("SISLES", "src")
number_of_aircraft = 2


if bCompareTCAS
    bReadSampleFromFile = true
end

if bGenerateSamplesToFile || bReadSampleFromFile
    initial_sample_filename = "initial.txt"
    transition_sample_filename = "transition.txt"
end

if bGenerateSamplesToFile
    number_of_initial_samples = 10
    number_of_transition_samples = 50

    aem = CorrAEM("$SISLES_PATH/Encounter/CorrAEMImpl/params/cor.txt", initial_sample_filename, number_of_initial_samples, transition_sample_filename, number_of_transition_samples)
    Encounter.generateEncountersToFile(aem)

    exit(0)
end

if bValidate
    number_of_initial_samples = 50
    number_of_transition_samples = 50

    aem = LLAEM(number_of_aircraft, "$SISLES_PATH/Encounter/LLAEMImpl/data/cor_enc.txt", number_of_initial_samples, ["$SISLES_PATH/Encounter/LLAEMImpl/data/cor_ac1.txt", "$SISLES_PATH/Encounter/LLAEMImpl/data/cor_ac2.txt"], [number_of_transition_samples, number_of_transition_samples])
else
    if bReadSampleFromFile
        aem = CorrAEM("$SISLES_PATH/Encounter/CorrAEMImpl/params/cor.txt", initial_sample_filename, transition_sample_filename)
        #Encounter.validate(aem)
    else
        aem = CorrAEM("$SISLES_PATH/Encounter/CorrAEMImpl/params/cor.txt")
    end
end

pr_1 = SimplePilotResponse()
pr_2 = SimplePilotResponse()

adm_1 = SimpleADM()
adm_2 = SimpleADM()

as = AirSpace(number_of_aircraft)

sr_1 = SimpleTCASSensor(1)
sr_2 = SimpleTCASSensor(2)

cas_1 = SimpleTCAS()
cas_2 = SimpleTCAS()


AC1_trajectory_ = Vector{Float64}[]
AC2_trajectory_ = Vector{Float64}[]

addObserver(adm_1, x -> push!(AC1_trajectory_, x))
addObserver(adm_2, x -> push!(AC2_trajectory_, x))


sim = TCASSimulator()

sim.parameters.em = aem
sim.parameters.pr = [pr_1, pr_2]
sim.parameters.dm = [adm_1, adm_2]
sim.parameters.wm = as
sim.parameters.sr = [sr_1, sr_2]
sim.parameters.cas = [cas_1, cas_2]
sim.parameters.number_of_aircraft = number_of_aircraft


if bValidate
    SimulationMatrix = Vector{Any}[]

    for i = 1:50
        simulate(sim)

        labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

        initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

        AC1_trajectory = zeros(length(AC1_trajectory_), 4)
        AC2_trajectory = zeros(length(AC2_trajectory_), 4)

        for i = 1:length(AC1_trajectory_)
            AC1_trajectory[i, :] = AC1_trajectory_[i]
            AC2_trajectory[i, :] = AC2_trajectory_[i]
        end

        AC1_trajectory_ll = getTrajectory(aem, 1)
        AC2_trajectory_ll = getTrajectory(aem, 2)

        push!(SimulationMatrix, {labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_ll, AC2_trajectory_ll})

        AC1_trajectory_ = Vector{Float64}[]
        AC2_trajectory_ = Vector{Float64}[]
    end

    save("validation.jld", "data", SimulationMatrix)

elseif bCompareTCAS
    simulate(sim, bTCAS = false, sample_number = sample_number)

    AC1_trajectory = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory[i, :] = AC1_trajectory_[i]
        AC2_trajectory[i, :] = AC2_trajectory_[i]
    end

    AC1_trajectory_ = Vector{Float64}[]
    AC2_trajectory_ = Vector{Float64}[]

    simulate(sim, bTCAS = true, sample_number = sample_number)

    AC1_trajectory_tcas = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory_tcas = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory_tcas[i, :] = AC1_trajectory_[i]
        AC2_trajectory_tcas[i, :] = AC2_trajectory_[i]
    end

    labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

    initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

    SimulationResult = {labels, initial, AC1_trajectory, AC2_trajectory, AC1_trajectory_tcas, AC2_trajectory_tcas}

    save("result_tcas.jld", "data", SimulationResult)

else
    if bReadSampleFromFile
        simulate(sim, bTCAS = bTCAS, sample_number = sample_number)
    else
        simulate(sim, bTCAS = bTCAS)
    end

    labels = ["A, L, chi(1: front, 2: back), beta(deg), C1, C2, hmd(ft), vmd(ft)", "time(sec), x_1(ft), y_1(ft), h_1(ft), x_2(ft), y_2(ft), h_2(ft)"]

    initial = [aem.A, aem.L, aem.geometry_at_TCA[1], aem.geometry_at_TCA[2], aem.C[1], aem.C[2], aem.geometry_at_TCA[3], aem.geometry_at_TCA[4]]

    AC1_trajectory = zeros(length(AC1_trajectory_), 4)
    AC2_trajectory = zeros(length(AC2_trajectory_), 4)

    for i = 1:length(AC1_trajectory_)
        AC1_trajectory[i, :] = AC1_trajectory_[i]
        AC2_trajectory[i, :] = AC2_trajectory_[i]
    end


    SimulationResult = {labels, initial, AC1_trajectory, AC2_trajectory}

    save("result.jld", "data", SimulationResult)
end


