zephyr_library()

zephyr_library_sources(
  vector_table.S
  reset.S
  fault.c
  exceptions.c
  irq_init.c
  reboot.c
  )
