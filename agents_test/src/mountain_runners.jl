# https://juliadynamics.github.io/Agents.jl/stable/examples/runners/

using Agents, Agents.Pathfinding
using Random
using FileIO

@agent Runner GridAgent{2} begin end

function initialize(map_url; goal = (128, 409), seed = 88)
    heightmap = floor.(Int, convert.(Float64, load(download(map_url))) * 255)
    space = GridSpace(size(heightmap); periodic = false)
    pathfinder = AStar(space; cost_metric = PenaltyMap(heightmap, MaxDistance{2}()))
    model = ABM(
        Runner,
        space;
        rng = MersenneTwister(seed),
        properties = Dict(:goal => goal, :pathfinder => pathfinder)
    )
    for _ in 1:10
        runner = add_agent!((rand(model.rng, 100:350), rand(model.rng, 50:200)), model)
        set_target!(runner, goal, model.pathfinder)
    end
    return model
end

agent_step!(agent, model) = move_along_route!(agent, model, model.pathfinder)

using InteractiveDynamics
using CairoMakie

map_url = 
    "https://raw.githubusercontent.com/JuliaDynamics/" *
    "JuliaDynamics/master/videos/agents/runners_heightmap.jpg"

model = initialize(map_url)

static_preplot!(ax, model) = scatter!(ax, model.goal; color = (:red, 50), marker = 'x')

abm_video(
    "runners.mp4",
    model,
    agent_step!;
    resolution = (700, 700),
    frames = 410,
    framerate = 45,
    ac = :black,
    as = 8,
    scatterkwargs = (strokecolor = :white, strokewidth = 2),
    heatarray = model -> penaltymap(model.pathfinder),
    heatkwargs = (colormap = :terrain,),
    static_preplot!
)