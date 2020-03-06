template <typename T>
T interp(T value, T in_min, T in_max, T out_min, T out_max) {
  auto interpolated = out_min + (out_max - out_min) * (value - in_min) / (in_max - in_min);
  return constrain(interpolated, out_min, out_max);
}

int transform2(float input, float in_low, float in_zero, float in_high, float out_low, float out_zero, float out_high) {
   if (input >= in_zero) {
    return interp(input, in_zero, in_high, out_zero, out_high);
   }
   return interp(input, in_low, in_zero, out_low, out_zero);
}
