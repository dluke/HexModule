#include "register_types.h"
#include "core/class_db.h"
#include "HexGrid.h"

void register_hexmodule_types() {
    ClassDB::register_class<HexGrid>();
}

void unregister_hexmodule_types() {
   //nothing to do here
}
