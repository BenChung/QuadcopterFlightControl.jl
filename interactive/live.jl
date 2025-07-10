using WGLMakie, Makie
using QuadcopterFlightControl

struct LiveFC 
    pts::Observable{Vector{Point{2, Float32}}}
    sl_xk::Slider
    sl_xv::Slider
end

function build_live_plot()
    pts = Observable([Point2f(0.0, 0.0), Point2f(0.0, 0.0)])
    f=Figure()
    mp=Axis(f[1,1:2])
    scatter!(mp, pts, color=[:red, :blue])
    sl_kx = Slider(f[2,1], range=0:0.01:10.0, startvalue=1.0, update_while_dragging=true)
    sl_kv = Slider(f[2,2], range=0:0.01:10.0, startvalue=1.0, update_while_dragging=true)
    display(f)
    return LiveFC(pts, sl_kx, sl_kv)
end

function QuadcopterFlightControl.update_interactive(l::LiveFC,tgt,real) 
    l.pts[] = [Point2f(tgt[1], tgt[2]), Point2f(real[1], real[2])]
    return QuadcopterFlightControl.GeometricControlParams(kx = l.sl_xk.value[], kv = l.sl_xv.value[])
end

lp = build_live_plot()
QuadcopterFlightControl.circle(lp)