// Icons: Lucide (ISC) for the general set, plus a few EUC-specific glyphs drawn in the same
// style: 24x24 viewBox, currentColor stroke, width 2, round caps and joins, no fill.

import type { SVGProps } from "react";

export {
  Activity,
  ArrowRight,
  BatteryMedium,
  Bell,
  BellOff,
  Bluetooth,
  BluetoothOff,
  Box,
  Camera,
  Check,
  ChevronDown,
  ChevronRight,
  CircleCheck,
  Command,
  Cpu,
  Crosshair,
  Eye,
  FileCode2,
  Flag,
  Gauge,
  Ghost,
  Info,
  Keyboard,
  LayoutGrid,
  Lock,
  MessageSquare,
  Microchip,
  OctagonAlert,
  Orbit,
  Palette,
  PanelBottom,
  PanelRight,
  Pause,
  Play,
  RefreshCw,
  RotateCcw,
  ScrollText,
  Search,
  ShieldCheck,
  SlidersHorizontal,
  Smartphone,
  StepForward,
  Send,
  ChevronUp,
  SquareTerminal,
  Thermometer,
  Timer,
  TriangleAlert,
  Waves,
  X,
  Zap,
} from "lucide-react";

export type IconProps = SVGProps<SVGSVGElement> & { size?: number };

function Svg({ size = 24, children, ...rest }: IconProps) {
  return (
    <svg
      xmlns="http://www.w3.org/2000/svg"
      width={size}
      height={size}
      viewBox="0 0 24 24"
      fill="none"
      stroke="currentColor"
      strokeWidth={2}
      strokeLinecap="round"
      strokeLinejoin="round"
      aria-hidden="true"
      {...rest}
    >
      {children}
    </svg>
  );
}

/** Electric unicycle seen from the side: tire, hub, shell and pedal. The Ride workspace. */
export function Unicycle(props: IconProps) {
  return (
    <Svg {...props}>
      <circle cx="12" cy="14" r="7" />
      <circle cx="12" cy="14" r="2" />
      <path d="M9 7V4.5a1.5 1.5 0 0 1 1.5-1.5h3A1.5 1.5 0 0 1 15 4.5V7" />
      <path d="M5 14h-2" />
      <path d="M21 14h-2" />
    </Svg>
  );
}

/** Two traces diverging from one point. The Compare workspace (A/B, first divergence). */
export function Diverge(props: IconProps) {
  return (
    <Svg {...props}>
      <path d="M3 12h6" />
      <path d="M9 12c3 0 4-6 8-6h4" />
      <path d="M9 12c3 0 4 6 8 6h4" />
      <circle cx="9" cy="12" r="1.5" />
    </Svg>
  );
}

/** Microcontroller with an arrow writing into it. The Flash workspace. */
export function ChipWrite(props: IconProps) {
  return (
    <Svg {...props}>
      <rect x="6" y="9" width="12" height="12" rx="2" />
      <path d="M12 2v8" />
      <path d="m9 7 3 3 3-3" />
      <path d="M9 21v2M15 21v2M3 13h3M3 17h3M18 13h3M18 17h3" />
    </Svg>
  );
}

/** App mark: wheel with a pitch tick. */
export function LogoMark(props: IconProps) {
  return (
    <Svg {...props}>
      <circle cx="12" cy="12" r="9" />
      <circle cx="12" cy="12" r="3.5" />
      <path d="M12 3v5.5" />
    </Svg>
  );
}
