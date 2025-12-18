# Hub Slicing Controller (2C)

Listens for semantic `SLICE_PROMOTE` triggers (mocked locally) and generates RRM commands for the gNB slice manager. Commands are sent via UDP; expiration rolls back to best-effort.
