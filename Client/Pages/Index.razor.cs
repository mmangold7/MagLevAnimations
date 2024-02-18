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
    private bool _isSimulationRunning;
    private int _divisions = 10;
    private int _voxelsPerDivisions = 10;
    private bool _showGravityField;
    private bool _showMagneticField;
    private bool _userDipoleApproximation = true;

    protected override async Task OnAfterRenderAsync(bool firstRender)
    {
        if (firstRender)
        {
            _simulationManager = new SimulationManager();

            _simulationManager.InitializeTwoMagnets();
            await InitializeVisualization(_simulationManager.SimulationExtents);
            await UpdateVisualization();
        }
    }

    private async Task InitializeVisualization(Vector3 extents) =>
        await JsRuntime.InvokeVoidAsync("Animations.initializeThreeJs", new
        {
            width = extents.X,
            height = extents.Y,
            depth = extents.Z
        });

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
    }

    private async Task ToggleVisualizationMode()
    {
        _showCoils = !_showCoils;
        await UpdateVisualization();
    }

    private async Task ToggleGravityField()
    {
        _showGravityField = !_showGravityField;
        _simulationManager.ShowGravityField = _showGravityField;
        _simulationManager.Divisions = _divisions;
        if(_showGravityField) _simulationManager.CalculateGravityField();
        await UpdateVisualization();
    }

    private async Task ToggleMagneticField()
    {
        _showMagneticField = !_showMagneticField;
        _simulationManager.ShowMagneticField = _showMagneticField;
        _simulationManager.Divisions = _divisions;
        if (_showMagneticField) _simulationManager.CalculateMagneticField();
        await UpdateVisualization();
    }

    private async Task ToggleApproximation()
    {
        _userDipoleApproximation = !_userDipoleApproximation;

        _simulationManager?.SwitchApproximationTo(_userDipoleApproximation
            ? SimulationMode.DipoleApproximation
            : SimulationMode.VoxelBased, _divisions, _voxelsPerDivisions);

        await UpdateVisualization();
    }

    private async Task UpdateVisualization()
    {
        var magnets = _simulationManager.Magnets.Select(ConvertMagnetToJsObject).ToArray();
        var gravityField = _showGravityField ? _simulationManager.GravityFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null;
        var magneticField = _showMagneticField ? _simulationManager.MagneticFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null;
        await JsRuntime.InvokeVoidAsync("Animations.updateThreeJsScene", magnets, _showCoils, gravityField, magneticField);
    }

    private object ConvertMagnetToJsObject(Magnet magnet) => new
    {
        position = new { magnet.Position.X, magnet.Position.Y, magnet.Position.Z },
        radius = magnet.Radius,
        length = magnet.Length,
        magnetization = new { magnet.Magnetization.X, magnet.Magnetization.Y, magnet.Magnetization.Z, },
    };

    private object ConvertFieldToJsObject(FieldVector vector) => new
    {
        position = new { vector.Position.X, vector.Position.Y, vector.Position.Z },
        direction = new { vector.Direction.X, vector.Direction.Y, vector.Direction.Z },
        magnitude = vector.Magnitude,
    };
}