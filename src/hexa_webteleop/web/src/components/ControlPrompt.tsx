interface Props {
  onTakeControl: () => void;
}

// Replaces the button grid in place while a controller owns /cmd_vel.
export default function ControlPrompt({ onTakeControl }: Props) {
  return (
    <div id="control-prompt">
      <p>By default, the hexapod is controlled by a game controller.</p>
      <button id="take-control-btn" onClick={onTakeControl}>
        Take control
      </button>
    </div>
  );
}
