import {
  createContext,
  useCallback,
  useContext,
  useEffect,
  useState,
} from "react";
import { createPortal } from "react-dom";
import type { ReactNode } from "react";

// Where a modal's DOM goes, held above the router like the socket is.
//
// A modal written inside a view is a child of that view's box, and a view box
// is the wrong parent for one: `#preset-view` is a grid in landscape, and a
// `position: fixed` child of a grid container is sized to the grid area it
// lands in rather than to the viewport — which left the preset switching dialog
// one row tall, pinned to the top of the screen and cut off by the view's
// `overflow: hidden`. Portalled onto <body>, above every view box and outside
// the `#app` flexbox too, so no orientation's layout can reach it.
const ModalContext = createContext<HTMLElement | null>(null);

export function ModalProvider({ children }: { children: ReactNode }) {
  // Made during the first render rather than in the effect below, so the host
  // is there for the first modal and no consumer sees a null one. React renders
  // into it while it is still detached and it shows once the effect attaches
  // it, which no modal is up early enough to notice.
  const [host] = useState(() => {
    const el = document.createElement("div");
    el.id = "modal-host";
    return el;
  });

  useEffect(() => {
    document.body.appendChild(host);
    return () => host.remove();
  }, [host]);

  return <ModalContext value={host}>{children}</ModalContext>;
}

interface ModalProps {
  id?: string;
  children: ReactNode;
}

// The modal shell — backdrop plus dialog box — which renders into the host
// above instead of where it is written. Handed back as a component rather than
// an open/close pair so a modal stays markup in the view that owns it: it goes
// up and comes down with the condition around it, and its contents are props
// of that view, with no effect to keep the two in step.
export function useModal() {
  const host = useContext(ModalContext);

  return useCallback(
    ({ id, children }: ModalProps) => {
      if (!host) throw new Error("useModal() outside <ModalProvider>");
      return createPortal(
        <div id={id} className="overlay">
          <div className="dialog">{children}</div>
        </div>,
        host,
      );
    },
    [host],
  );
}
