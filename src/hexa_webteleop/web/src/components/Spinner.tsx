import { LoaderCircle } from "lucide-react";

// Busy indicator for the dialogs: the lucide loader ring, spun by CSS. Nothing
// ticks in JS, so mounting and unmounting one costs no timer and two on screen
// at once stay in step.
export default function Spinner({ size = 30 }: { size?: number }) {
  return <LoaderCircle className="spinner" size={size} aria-hidden />;
}
