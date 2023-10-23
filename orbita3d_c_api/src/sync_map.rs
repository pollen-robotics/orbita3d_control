use std::{cell::UnsafeCell, collections::HashMap};

pub struct SyncMap<K, V>(UnsafeCell<HashMap<K, V>>);

unsafe impl<K: Sync + std::hash::Hash + Eq, V> Sync for SyncMap<K, V> {}

impl<K: std::hash::Hash + Eq, V> SyncMap<K, V> {
    pub fn new() -> Self {
        Self(UnsafeCell::new(HashMap::new()))
    }

    pub fn insert(&self, key: K, value: V) -> Option<V> {
        unsafe { (*self.0.get()).insert(key, value) }
    }

    pub fn get(&self, key: &K) -> Option<&V> {
        unsafe { (*self.0.get()).get(key) }
    }

    pub fn get_mut(&self, key: &K) -> Option<&mut V> {
        unsafe { (*self.0.get()).get_mut(key) }
    }
}
