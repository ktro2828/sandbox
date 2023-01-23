package main

import "testing"

func TestWantError(t *testing.T, err error, wantError bool) {
	t.Helper()

	if err != nil && !wantError {
		t.Errorf("got an error %v, want nothing happened", err)
	} else if err == nil && wantError {
		t.Error("got nothing happened, although want an error")
	}
}
