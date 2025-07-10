
using QuadcopterFlightControl

function @main(args::Vector{String})::Cint
    println(Core.stdout, "Running flight controller")
    
    QuadcopterFlightControl.circle(nothing)

    return zero(Cint)
end
