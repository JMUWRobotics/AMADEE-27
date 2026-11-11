# Power Management and MPPT Control Specification

## 1. Voltage Thresholds & Safety
- **$\ge 51\text{ V}$:** MPPT charging only.
- **$\le 48\text{ V}$:** System Error / Critical Low Voltage.

---

## 2. MPPT Power Limitation & Discovery
When the MPPT throttles to limit power, the true potential power output of the solar panels is unknown. The following protocol outlines how the system tests and adapts to these limits:

1. **Probe Limits:** When higher power is requested, slightly increase the system power output to test the physical boundaries.
2. **Throttled State Response:** If the MPPT flag indicates that it is being throttled:
   - Increase the maximum provided power according to the formula:
     $$\Delta P = 80\% \times V_{\text{battery}} \times I_{\text{discharge}}$$
   - If the MPPT continues to generate solar power while the flag remains in the throttled state, repeat Step 1.
3. **Unthrottled Recovery:** Once the MPPT flag changes to "not throttled", step back by one power increment to stabilize operation.

---

## 3. Multi-Robot Power Distribution
- **Fair Share:** Every robot is entitled to an equal portion of the available power.
- **Surplus Allocation:** Any excess energy is distributed equally among the remaining active demands.