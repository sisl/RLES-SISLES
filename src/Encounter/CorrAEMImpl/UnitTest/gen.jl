#using Gallium
using SISLES

const PARAMFILE = Pkg.dir("SISLES/src/Encounter/CorrAEMImpl/params/cor.txt")
const INITFILE = "initial.txt"
const TRANFILE = "transition.txt"
const NSAMPLES = 100
const NTRANS = 50

function run_validate(; parameter_file=PARAMFILE, initial_sample_filename=INITFILE,    
    number_of_initial_samples=NSAMPLES, transition_sample_filename=TRANFILE,    
    number_of_transition_samples=NTRANS)

    aem = CorrAEM(parameter_file, initial_sample_filename, number_of_initial_samples,
        transition_sample_filename, number_of_transition_samples)

    #Gallium.breakpoint(CorrAEMImpl.validate_initial)

    CorrAEMImpl.validate_initial(aem)
    CorrAEMImpl.validate_transition(aem)

    #Encounter.generateEncountersToFile(aem)
    #Gallium.breakpoint(CorrAEMImpl.bn_sample)
end


