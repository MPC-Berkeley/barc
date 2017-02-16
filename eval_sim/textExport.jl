# Michael Garstka
# 02/09/2017

# This function is used to export measurement data and kalman filtered data to text files.

# HowTo Use This:
# 1. Include file using Julia command line
# 2. Use function TextExport.export2txt(code string of the JLD data file) or type "using TextExport" to be able to call just "export2txt(code)"
# 3. You need both the output-record- and the output-LMPC- file for this to work

module TextExport

export export2txt

using JLD

function export2txt(code::AbstractString)
    log_path_record = "$(homedir())/simulations/output-record-$(code).jld"
    log_path_LMPC   = "$(homedir())/simulations/output-LMPC-$(code).jld"

    # open files and load into variables
    d_rec = load(log_path_record)
    pos_info = d_rec["pos_info"]
    d_lmpc = load(log_path_LMPC)
    t    = d_lmpc["t"]

    # extract data
    s = pos_info.z[:,1]
    ey = pos_info.z[:,2]
    epsi = pos_info.z[:,3]
    v = pos_info.z[:,4]
    x = pos_info.z[:,6]
    y = pos_info.z[:,7]
    vx = pos_info.z[:,8]
    vy = pos_info.z[:,9]
    psi = pos_info.z[:,10]
    psiDot = pos_info.z[:,11]

    x_raw = pos_info.z[:,12]
    y_raw = pos_info.z[:,13]
    psi_raw = pos_info.z[:,14]
    v_raw = pos_info.z[:,15]
    psi_drift = pos_info.z[:,16]
    a_x = pos_info.z[:,17]
    a_y = pos_info.z[:,18]
    a_x_raw = pos_info.z[:,19]
    a_y_raw = pos_info.z[:,20]

    # determine index when car starts moving (first time when LMPC node time is logged)
    dataStart = findfirst(f->f>t[1],pos_info.t) #time when LMPC node starts
    dataEnd = length(s)

    # write relevant data rowwise into text file
    open("BARC_measurement_data_$(code).txt", "w") do f
     write(f, "s,ey,epsi,v,x,y,vx,vy,psi,psiDot,x_raw,y_raw,psi_raw,v_raw,psi_drift,a_x,a_y,a_x_raw,a_y_raw\n")
        for k = dataStart:1:dataEnd
            write(f, "$(s[k]),$(ey[k]),$(epsi[k]),$(v[k]),$(x[k]),$(y[k]),$(vx[k]),$(vy[k]),$(psi[k]),$(psiDot[k]),$(x_raw[k]),$(y_raw[k]),$(psi_raw[k]),$(v_raw[k]),$(psi_drift[k]),$(a_x[k]),$(a_y[k]),$(a_x_raw[k]),$(a_y_raw[k])\n")
        end
     end
     return nothing
end
end # END MODULE