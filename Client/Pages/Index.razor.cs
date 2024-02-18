using Microsoft.AspNetCore.Components;
using Microsoft.JSInterop;
using System.Numerics;
using Animations.Shared;

namespace Animations.Client.Pages;

public partial class Index : ComponentBase
{

    [Inject]
    protected IJSRuntime? JsRuntime { get; set; }
    private SimulationManager? _simulationManager;
    private bool _showCoils;
    private bool _isSimulationRunning = false;

    protected override async Task OnAfterRenderAsync(bool firstRender)
    {
        if (firstRender)
        {
            _simulationManager = new SimulationManager();
            await InitializeVisualization(_simulationManager.SimulationExtents);
            await UpdateVisualization();
        }
    }

    private async Task InitializeVisualization(Vector3 extents)
    {
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs", new
        {
            width = extents.X,
            height = extents.Y,
            depth = extents.Z
        });
    }

    private async Task ToggleSimulation()
    {
        _isSimulationRunning = !_isSimulationRunning;

        if (_isSimulationRunning)
        {
            for (int i = 0; _isSimulationRunning && i < 1000; i++)
            {
                _simulationManager.UpdateSimulation();
                await UpdateVisualization();
                await Task.Delay(100);
            }
        }
        // If _isSimulationRunning is false, the loop exits, effectively pausing the simulation
    }

    private async Task ToggleVisualizationMode()
    {
        _showCoils = !_showCoils;
        await UpdateVisualization();
    }

    private async Task UpdateVisualization()
    {
        var magnets = new[] {
            ConvertMagnetToJsObject(_simulationManager.TargetMagnet),
            ConvertMagnetToJsObject(_simulationManager.FixedMagnet)
        };

        await JsRuntime.InvokeVoidAsync("Animations.updateThreeJsScene", magnets, _showCoils);
    }

    private object ConvertMagnetToJsObject(Magnet magnet)
    {
        return new
        {
            position = new { magnet.Position.X, magnet.Position.Y, magnet.Position.Z },
            radius = magnet.Radius,
            length = magnet.Length,
            magnetization = new { magnet.Magnetization.X, magnet.Magnetization.Y, magnet.Magnetization.Z}
            //color = 0xff0000
        };
    }
}