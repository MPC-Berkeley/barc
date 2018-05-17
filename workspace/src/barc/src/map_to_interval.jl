using PyPlot

val = linspace(- 2 * pi, 0 * pi, 720)
val_orig = linspace(-pi, pi, 720)

function map_to_pi(val)
	return (val + pi) .% (2 * pi) - pi
end

fig = figure("Mapping")
ax_sin = fig[:add_subplot](1, 2, 1)
ax_cos = fig[:add_subplot](1, 2, 2)

ax_sin[:plot](val_orig, sin(val_orig), "r-")
ax_sin[:plot](map_to_pi(val), sin(map_to_pi(val)), "b-")

ax_cos[:plot](val_orig, cos(val_orig), "r-")
ax_cos[:plot](map_to_pi(val), cos(map_to_pi(val)), "b-")

plt[:show]()

