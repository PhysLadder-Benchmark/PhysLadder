from .runtime import configure_windows_storage

configure_windows_storage()

from .cli import main

raise SystemExit(main())
