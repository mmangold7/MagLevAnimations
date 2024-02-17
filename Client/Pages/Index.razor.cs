using Microsoft.AspNetCore.Components;
using Microsoft.JSInterop;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{

    [Inject]
    protected IJSRuntime JSRuntime { get; set; }

    private SimulationManager simulationManager;
    private bool showLoops = false; // Controls the visualization mode

    protected override async Task OnInitializedAsync()
    {
        simulationManager = new SimulationManager();
        await UpdateVisualization(); // Initial visualization
    }

    private async Task StartSimulation()
    {
        for (int i = 0; i < 1000; i++)
        {
            simulationManager.UpdateSimulation();
            await UpdateVisualization();
            await Task.Delay(100);
        }
    }

    private async Task ToggleVisualization()
    {
        showLoops = !showLoops;
        await UpdateVisualization();
    }

    private async Task UpdateVisualization()
    {
        // Convert Magnet properties to a format suitable for JS interop
        var magnetInfo = new
        {
            position = new { simulationManager.TargetMagnet.Position.X, simulationManager.TargetMagnet.Position.Y, simulationManager.TargetMagnet.Position.Z },
            radius = simulationManager.TargetMagnet.Radius,
            length = simulationManager.TargetMagnet.Length,
            color = 0xff0000 // Example color, adjust as needed
        };

        await JSRuntime.InvokeVoidAsync("updateVisualization", magnetInfo, showLoops);
    }
}