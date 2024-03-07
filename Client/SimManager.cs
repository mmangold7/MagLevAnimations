using System.Numerics;
using Animations.Client.Extensions;
using Animations.Client.Models;
using Animations.Client.Models.Parameters;

namespace Animations.Client;

public class SimManager
{
    private readonly PhysicsSimulation.PhysicsSimulation _physicsSimulator;

    private readonly Dictionary<int, FieldVector> _previousGravityFieldVectors = new();
    private readonly Dictionary<int, FieldVector> _previousMagneticFieldVectors = new();

    private List<FieldVector> _gravityFieldVectors = new();
    private List<FieldVector> _magneticFieldVectors = new();

    public SimManager(SimulationParameters initialParameters)
    {
        _physicsSimulator = new PhysicsSimulation.PhysicsSimulation(initialParameters.SimulationExtents);
        _physicsSimulator.UpdateParameters(initialParameters);
    }

    public void UpdateSimulation(SimulationParameters parameters)
    {
        UpdateSimulationState(parameters);
        RecalculateFields(parameters);
    }

    private void UpdateSimulationState(SimulationParameters newParameters)
    {
        _physicsSimulator.UpdateParameters(newParameters);
        _physicsSimulator.UpdateMagnetsPositionsBasedOnMode();
        _physicsSimulator.DetectAndResolveCollisions();
    }

    public void RecalculateFields(SimulationParameters parameters)
    {
        if (parameters.ShowGravityField) Profiling.RunWithClockingLog(
            () => _gravityFieldVectors = Fields.CalculateGravityField(
                parameters.Gravity, _previousGravityFieldVectors, parameters.SimulationExtents, parameters.Divisions));

        if (parameters.ShowMagneticField) Profiling.RunWithClockingLog(
            () => _magneticFieldVectors = Fields.CalculateMagneticField(
                _physicsSimulator.Magnets, parameters.SimulationMode, _previousMagneticFieldVectors, parameters.SimulationExtents, parameters.Divisions));
    }

    public SimulationStateForVisualization GetSimulationState(bool showGravityField, bool showMagneticField)
    {
        return new SimulationStateForVisualization
        {
            Magnets = _physicsSimulator.Magnets.Select(ConvertMagnetToJsObject).ToArray(),
            GravityFieldData = showGravityField ? _gravityFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MagneticFieldData = showMagneticField ? _magneticFieldVectors.Select(ConvertFieldToJsObject).ToArray() : null,
            MiddleSliceOfFieldVectors = Get2DMiddleSliceOfFieldVectors(_physicsSimulator.Extents).Select(ConvertFieldToJsObject).ToArray()
        };
    }

    private static object ConvertMagnetToJsObject(Magnet magnet) => new
    {
        position = new { magnet.Position.X, magnet.Position.Y, magnet.Position.Z },
        radius = magnet.Radius,
        length = magnet.Length,
        magnetization = new { magnet.Magnetization.X, magnet.Magnetization.Y, magnet.Magnetization.Z, },
    };

    private static object ConvertFieldToJsObject(FieldVector vector) => new
    {
        position = new { vector.Position.X, vector.Position.Y, vector.Position.Z },
        direction = new { vector.Direction.X, vector.Direction.Y, vector.Direction.Z },
        magnitude = vector.Magnitude,
        index = vector.Index
    };

    public List<FieldVector> Get2DMiddleSliceOfFieldVectors(Vector3 extents)
    {
        var middleY = extents.Y / 2;
        var sliceFieldVectors = new List<FieldVector>();

        var closestY = _magneticFieldVectors?
            .MinBy(vector => Math.Abs(vector.Position.Y - middleY))
            ?.Position.Y;

        if (_magneticFieldVectors != null)
            sliceFieldVectors.AddRange(_magneticFieldVectors
                .Where(vector => Math.Abs((float)(vector.Position.Y - closestY)) < 1e-6));

        return sliceFieldVectors;
    }
}