include("config_2.jl")
include("track.jl")

track = Track()
init!(track)

println(get_curvature(track, 6.49))
println(get_curvature(track, 6.5))
println(get_curvature(track, 6.501))
println(get_curvature(track, 6.6))