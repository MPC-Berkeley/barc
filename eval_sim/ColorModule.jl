# This module is used to achieve a higher variety of different colors for plotting.
# 20 colors are listed in the colors-field of the ColorManager type.
# getColor Method cycles through the array and starts at the begining after reaching the end
# Michael Garstka, 11/24/2016

module ColorModule
  export ColorManager, getColor, resetColorCycle, toggle_mum

  type ColorManager
    colors::Array{String}
    mum_colors::Array{String}
    currentColorIndex::Int64
    mum_active::Bool

    function ColorManager()
      colordefs=["#3b44ba",
      "#c8ce24",
      "#a756de",
      "#0ac753",
      "#ff53c4",
      "#01c3ba",
      "#f5218a",
      "#4cd5ff",
      "#f74b3a",
      "#016ed6",
      "#de8500",
      "#8a1e95",
      "#6c4d08",
      "#aca0ff",
      "#a2172f",
      "#abaae1",
      "#ffb38d",
      "#83317e",
      "#9c6f4b",
      "#ff91d1"]
      mum_colors=["#dc214d","#008c00","#0094de","#660066","#ff6600","#37c8ab"]
      new(colordefs,mum_colors,1,false)
    end
end

function getColor(CM::ColorModule.ColorManager)::String

  if CM.mum_active
    used_colors = CM.mum_colors
  else
    used_colors = CM.colors
  end

  totalNumColors = length(used_colors)

  if CM.currentColorIndex > totalNumColors
    CM.currentColorIndex = 1
  end

  nextColorString = used_colors[CM.currentColorIndex]
  CM.currentColorIndex = CM.currentColorIndex + 1

  return nextColorString
end

function resetColorCycle(CM::ColorModule.ColorManager)
  CM.currentColorIndex = 1
  return nothing
end

function toggle_mum(CM::ColorModule.ColorManager)
  if CM.mum_active
    CM.mum_active = false
  else
    CM.mum_active = true
  end
  CM.currentColorIndex = 1
  return nothing
end

end #END MODULE
