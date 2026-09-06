import { Gamepad } from "lucide-react";

interface Props {
  onTakeControl: () => void;
}

// The whole Control view while a controller owns /cmd_vel: every input that
// view has feeds a /cmd_vel this page is not the source of, so none of them is
// offered, and what is left is the one thing the page can still ask for.
export default function ControlPrompt({ onTakeControl }: Props) {
  return (
    <div id="control-prompt">
      <p>By default, the hexapod is controlled by a game controller.</p>
      <button id="take-control-btn" onClick={onTakeControl}>
        <Gamepad aria-hidden />
        Control from here
      </button>
    </div>
  );
}
